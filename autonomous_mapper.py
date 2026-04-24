#!/usr/bin/env python3
"""
autonomous_mapper.py — High-level mapping executive for Spot + RTAB-Map.

Drives Spot in a structured pattern designed to maximise RTAB-Map loop closure
opportunities: short forward segments, landmark pauses with look motions, and
deliberate revisits to earlier poses.

Does NOT require Nav2 — sends velocity commands directly to /spot/cmd_vel.

Prerequisites (must be running before starting this node):
  T1: ros2 launch spot_driver spot_driver.launch.py spot_name:=spot config_file:=/root/spot_config.yaml
  T2: python3 /root/tf_keep_alive.py
  T3: bash /root/start_rtabmap_map.sh

Launch this node:
  python3 -u /root/autonomous_mapper.py --ros-args \
    -p max_duration_sec:=300.0 \
    -p segment_distance:=0.75 \
    -p revisit_interval:=4

State machine flow:
  STARTUP → VERIFY_SYSTEM → START_MAPPING → INITIAL_OBSERVE
    → (MOVE_SEGMENT → LANDMARK_PAUSE → REVISIT_CHECK
       → [REVISIT_PREVIOUS →] MOVE_SEGMENT ...) × N
    → MAP_QUALITY_CHECK → SAVE_MAP → SWITCH_TO_LOCALIZATION → DONE

  At any point:  → RECOVERY (obstacle / timeout)
                 → ABORT    (system fault)
"""

import math
import time
from enum import Enum, auto
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

try:
    from rtabmap_msgs.msg import Info as RtabmapInfo
    _HAVE_RTABMAP = True
except ImportError:
    _HAVE_RTABMAP = False


# ──────────────────────────────────────────────────────────────────────────────
# State definitions
# ──────────────────────────────────────────────────────────────────────────────

class State(Enum):
    STARTUP                = auto()
    VERIFY_SYSTEM          = auto()
    START_MAPPING          = auto()
    INITIAL_OBSERVE        = auto()
    MOVE_SEGMENT           = auto()
    LANDMARK_PAUSE         = auto()
    REVISIT_CHECK          = auto()
    REVISIT_PREVIOUS       = auto()
    MAP_QUALITY_CHECK      = auto()
    SAVE_MAP               = auto()
    SWITCH_TO_LOCALIZATION = auto()
    DONE                   = auto()
    RECOVERY               = auto()
    ABORT                  = auto()


# ──────────────────────────────────────────────────────────────────────────────
# Lightweight 2D pose extracted from odometry
# ──────────────────────────────────────────────────────────────────────────────

class Pose2D:
    __slots__ = ('x', 'y', 'yaw')

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.x   = x
        self.y   = y
        self.yaw = yaw

    def distance_to(self, other: 'Pose2D') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def bearing_to(self, other: 'Pose2D') -> float:
        """Absolute bearing (rad) from this pose to other, in the odom frame."""
        return math.atan2(other.y - self.y, other.x - self.x)

    def copy(self) -> 'Pose2D':
        return Pose2D(self.x, self.y, self.yaw)

    def __repr__(self) -> str:
        return f'Pose2D(x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f}°)'


def quat_to_yaw(q) -> float:
    """Extract yaw (rad) from a geometry_msgs/Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


# ──────────────────────────────────────────────────────────────────────────────
# Main node
# ──────────────────────────────────────────────────────────────────────────────

class AutonomousMapper(Node):

    # ── construction ──────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('autonomous_mapper')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic',         '/spot/cmd_vel')
        self.declare_parameter('forward_speed',          0.15)   # m/s — slow for visual SLAM
        self.declare_parameter('angular_speed',          0.30)   # rad/s
        self.declare_parameter('segment_distance',       0.75)   # m per forward segment
        self.declare_parameter('pause_duration',         2.0)    # s at each landmark pause
        self.declare_parameter('observe_angle_deg',     30.0)    # degrees for left/right look
        self.declare_parameter('revisit_interval',       4)      # revisit every N segments
        self.declare_parameter('max_duration_sec',     300.0)    # total mapping time budget
        self.declare_parameter('min_nodes_for_quality', 30)      # RTAB-Map WM size target
        self.declare_parameter('min_loop_closures',      1)      # desired loop closures
        self.declare_parameter('obstacle_distance',      0.65)   # m — stop if depth < this
        self.declare_parameter('revisit_tolerance',      0.50)   # m — "arrived at revisit target"
        self.declare_parameter('tick_hz',               10.0)    # state machine update rate

        p = self.get_parameter
        self._cmd_topic   = str(p('cmd_vel_topic').value)
        self._fwd_speed   = float(p('forward_speed').value)
        self._ang_speed   = float(p('angular_speed').value)
        self._seg_dist    = float(p('segment_distance').value)
        self._pause_dur   = float(p('pause_duration').value)
        self._obs_angle   = math.radians(float(p('observe_angle_deg').value))
        self._revisit_ivl = int(p('revisit_interval').value)
        self._max_dur     = float(p('max_duration_sec').value)
        self._min_nodes   = int(p('min_nodes_for_quality').value)
        self._min_loops   = int(p('min_loop_closures').value)
        self._obs_dist    = float(p('obstacle_distance').value)
        self._rev_tol     = float(p('revisit_tolerance').value)

        # ── publishers ────────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)

        # ── subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, '/spot/odometry', self._odom_cb, 10)

        # Depth image for forward obstacle detection (16UC1, values in mm)
        self.create_subscription(
            Image, '/spot/depth_registered/frontleft/image', self._depth_cb, 1)

        if _HAVE_RTABMAP:
            self.create_subscription(
                RtabmapInfo, '/rtabmap/info', self._info_cb, 10)
        else:
            self.get_logger().warn(
                'rtabmap_msgs not importable — RTAB-Map monitoring disabled. '
                'Source your ROS2 workspace before running this node.')

        # ── service clients ───────────────────────────────────────────────────
        self._backup_cli = self.create_client(Empty, '/rtabmap/rtabmap/backup')

        # ── odometry ──────────────────────────────────────────────────────────
        self._pose          = Pose2D()
        self._odom_received = False
        self._odom_stamp    = 0.0       # monotonic time of last odom msg

        # ── RTAB-Map state ────────────────────────────────────────────────────
        self._rtab_received = False
        self._rtab_stamp    = 0.0
        self._rtab_wm       = 0         # working memory size (≈ node count)
        self._rtab_loops    = 0         # total loop closures detected this session

        # ── depth / obstacle ──────────────────────────────────────────────────
        self._min_fwd_depth = float('inf')   # metres, updated per depth frame

        # ── state machine ─────────────────────────────────────────────────────
        self._state      = State.STARTUP
        self._sub_phase  = 0
        self._phase_t0   = self._now()  # wall time when current sub-phase began

        # ── mission memory ────────────────────────────────────────────────────
        self._mission_t0       = 0.0
        self._start_pose: Optional[Pose2D] = None
        self._stored_poses: List[Pose2D]   = []  # landmark poses saved for revisit
        self._segments_done    = 0
        self._revisit_target: Optional[Pose2D] = None

        # ── scratch variables (set before use in each state) ──────────────────
        self._move_start       = Pose2D()   # pose recorded at start of a motion
        self._look_base_yaw    = 0.0        # yaw before a look sequence begins
        self._recovery_turn_yaw = 0.0       # target yaw for recovery turn

        # ── recovery ──────────────────────────────────────────────────────────
        self._recovery_reason = ''
        self._recovery_return = State.MOVE_SEGMENT

        # ── throttled-warn tracker ────────────────────────────────────────────
        self._last_warn_t = 0.0

        # ── main timer ────────────────────────────────────────────────────────
        period = 1.0 / float(p('tick_hz').value)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'autonomous_mapper ready\n'
            f'  cmd_topic={self._cmd_topic}\n'
            f'  speed={self._fwd_speed} m/s  angular={self._ang_speed} rad/s\n'
            f'  segment={self._seg_dist} m  pause={self._pause_dur} s\n'
            f'  revisit every {self._revisit_ivl} segments\n'
            f'  budget={self._max_dur:.0f} s  min_nodes={self._min_nodes}')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose.x   = p.x
        self._pose.y   = p.y
        self._pose.yaw = quat_to_yaw(msg.pose.pose.orientation)
        self._odom_received = True
        self._odom_stamp    = self._now()

    def _info_cb(self, msg: 'RtabmapInfo'):
        # wm_state is the list of node IDs in working memory; its length is WM size
        self._rtab_wm = len(msg.wm_state)

        # loop_closure_id > 0 when a loop closure was detected this frame
        if msg.loop_closure_id > 0:
            self._rtab_loops += 1
            self.get_logger().info(
                f'[RTAB-Map] *** Loop closure detected! ***  '
                f'id={msg.loop_closure_id}  '
                f'total_closures={self._rtab_loops}  '
                f'wm_nodes={self._rtab_wm}')

        self._rtab_received = True
        self._rtab_stamp    = self._now()

    def _depth_cb(self, msg: Image):
        """
        Extract minimum depth in the forward-facing centre strip.
        Image is 16UC1 (uint16), values in millimetres.
        We check the centre third of the image width across all rows.
        """
        h, w = msg.height, msg.width
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
        except ValueError:
            return

        # Central vertical strip — covers what Spot walks into
        cx1 = w // 3
        cx2 = 2 * w // 3
        centre = arr[:, cx1:cx2]
        valid  = centre[centre > 0]   # 0 = no reading (too close / invalid)
        self._min_fwd_depth = (float(np.min(valid)) / 1000.0
                               if len(valid) > 0 else float('inf'))

    # ── utilities ─────────────────────────────────────────────────────────────

    def _now(self) -> float:
        """Monotonic wall clock in seconds (independent of Spot's clock offset)."""
        return time.monotonic()

    def _stop(self):
        """Publish a zero-velocity command."""
        self._cmd_pub.publish(Twist())

    def _drive(self, lin: float, ang: float):
        """Publish a velocity command."""
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self._cmd_pub.publish(msg)

    def _transition(self, new_state: State):
        """
        Change state, reset sub_phase to 0 and refresh the phase timer.
        Always log the transition so the operator can follow progress.
        """
        self.get_logger().info(
            f'  ▶  {self._state.name}  →  {new_state.name}')
        self._state     = new_state
        self._sub_phase = 0
        self._phase_t0  = self._now()

    def _warn_throttled(self, text: str, period: float = 3.0):
        """Log a warning at most once per `period` seconds."""
        now = self._now()
        if now - self._last_warn_t >= period:
            self.get_logger().warn(text)
            self._last_warn_t = now

    def _obstacle_ahead(self) -> bool:
        """True if the forward depth reading is within the stop threshold."""
        return self._min_fwd_depth < self._obs_dist

    def _turn_toward_yaw(self, target_yaw: float, tol_deg: float = 3.0) -> bool:
        """
        Publish rotation commands to turn toward target_yaw.
        Returns True (and stops the robot) when within tolerance.
        Caller must NOT publish additional commands on the same tick.
        """
        err = wrap_angle(target_yaw - self._pose.yaw)
        if abs(err) < math.radians(tol_deg):
            self._stop()
            return True
        self._drive(0.0, math.copysign(self._ang_speed, err))
        return False

    # ── state handlers ────────────────────────────────────────────────────────

    def _do_startup(self, _now: float):
        """One-shot: just log and move on."""
        self.get_logger().info('STARTUP: node active')
        self._transition(State.VERIFY_SYSTEM)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_verify_system(self, now: float):
        """
        Block until odometry is live. RTAB-Map info is expected but not fatal if
        absent — we warn and continue after 10 s so a late-starting RTAB-Map does
        not abort the mission.

        Times out to ABORT after 30 s total.
        """
        TOTAL_TIMEOUT  = 30.0
        RTAB_GRACE     = 10.0

        if self._sub_phase == 0:
            self.get_logger().info('VERIFY_SYSTEM: checking system health...')
            self._sub_phase = 1

        elapsed = now - self._phase_t0

        if elapsed > TOTAL_TIMEOUT:
            self.get_logger().error('VERIFY_SYSTEM: timed out — aborting')
            self._transition(State.ABORT)
            return

        # 1) Odometry must be fresh
        odom_ok = self._odom_received and (now - self._odom_stamp < 2.0)
        if not odom_ok:
            self._warn_throttled('VERIFY_SYSTEM: waiting for /spot/odometry...')
            return

        # 2) RTAB-Map info — give it RTAB_GRACE seconds, then warn and continue
        if not self._rtab_received:
            if elapsed < RTAB_GRACE:
                self._warn_throttled(
                    'VERIFY_SYSTEM: waiting for /rtabmap/info...', period=4.0)
                return
            self.get_logger().warn(
                'VERIFY_SYSTEM: /rtabmap/info not received after '
                f'{RTAB_GRACE:.0f} s — continuing without RTAB-Map feedback')

        self.get_logger().info('VERIFY_SYSTEM: all systems healthy ✓')
        self._transition(State.START_MAPPING)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_start_mapping(self, now: float):
        """Record mission start pose and begin the time budget clock."""
        self._start_pose    = self._pose.copy()
        self._mission_t0    = now
        self._segments_done = 0
        self._stored_poses  = [self._start_pose.copy()]  # always keep start for revisit
        self.get_logger().info(
            f'START_MAPPING: start_pose={self._start_pose}  '
            f'budget={self._max_dur:.0f} s')
        self._transition(State.INITIAL_OBSERVE)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_initial_observe(self, now: float):
        """
        Before moving, let RTAB-Map accumulate distinctive frames of the starting
        area by standing still then performing a left-right look sweep.

        Sub-phases:
          0: entry — stop, record base yaw
          1: static pause (3 s)
          2: turn to base_yaw + obs_angle  (look left)
          3: pause at left position (1 s)
          4: return to base_yaw
          5: pause at centre (0.5 s)
          6: turn to base_yaw − obs_angle  (look right)
          7: pause at right position (1 s)
          8: return to base_yaw
          9: final pause (0.5 s) → MOVE_SEGMENT
        """
        STATIC = 3.0   # s — initial static observation
        LOOK   = 1.0   # s — pause at each look direction
        CENTRE = 0.5   # s — pause when centred

        if self._sub_phase == 0:
            self._stop()
            self._look_base_yaw = self._pose.yaw
            self.get_logger().info(
                'INITIAL_OBSERVE: standing still while RTAB-Map initialises...')
            self._sub_phase = 1

        elif self._sub_phase == 1:   # static pause
            if now - self._phase_t0 >= STATIC:
                self.get_logger().info('INITIAL_OBSERVE: looking left')
                self._sub_phase = 2

        elif self._sub_phase == 2:   # turn left
            if self._turn_toward_yaw(self._look_base_yaw + self._obs_angle):
                self._phase_t0  = now
                self._sub_phase = 3

        elif self._sub_phase == 3:   # pause at left
            if now - self._phase_t0 >= LOOK:
                self.get_logger().info('INITIAL_OBSERVE: returning to centre')
                self._sub_phase = 4

        elif self._sub_phase == 4:   # return to centre
            if self._turn_toward_yaw(self._look_base_yaw):
                self._phase_t0  = now
                self._sub_phase = 5

        elif self._sub_phase == 5:   # pause at centre
            if now - self._phase_t0 >= CENTRE:
                self.get_logger().info('INITIAL_OBSERVE: looking right')
                self._sub_phase = 6

        elif self._sub_phase == 6:   # turn right
            if self._turn_toward_yaw(self._look_base_yaw - self._obs_angle):
                self._phase_t0  = now
                self._sub_phase = 7

        elif self._sub_phase == 7:   # pause at right
            if now - self._phase_t0 >= LOOK:
                self.get_logger().info('INITIAL_OBSERVE: returning to centre')
                self._sub_phase = 8

        elif self._sub_phase == 8:   # return to centre
            if self._turn_toward_yaw(self._look_base_yaw):
                self._phase_t0  = now
                self._sub_phase = 9

        elif self._sub_phase == 9:   # final pause
            if now - self._phase_t0 >= CENTRE:
                self.get_logger().info('INITIAL_OBSERVE: complete — starting forward motion')
                self._transition(State.MOVE_SEGMENT)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_move_segment(self, now: float):
        """
        Drive forward exactly _seg_dist metres using odometry feedback.

        Stops and enters RECOVERY if an obstacle is detected closer than
        _obs_dist, or if the move takes more than 3× the expected time.

        Sub-phases:
          0: record start pose
          1: drive forward; check distance, obstacles, timeout
             → LANDMARK_PAUSE on success
             → RECOVERY on obstacle
             → LANDMARK_PAUSE on timeout (don't abort — wall ahead is common)
        """
        TIMEOUT = (self._seg_dist / max(self._fwd_speed, 0.05)) * 3.0

        if self._sub_phase == 0:
            self._move_start = self._pose.copy()
            self.get_logger().info(
                f'MOVE_SEGMENT #{self._segments_done + 1}: '
                f'target={self._seg_dist:.2f} m  from={self._move_start}')
            self._sub_phase = 1

        elif self._sub_phase == 1:
            dist = self._pose.distance_to(self._move_start)

            # Obstacle check — highest priority
            if self._obstacle_ahead():
                self._stop()
                self.get_logger().warn(
                    f'MOVE_SEGMENT: obstacle at {self._min_fwd_depth:.2f} m — recovering')
                self._recovery_reason = 'obstacle'
                self._recovery_return = State.LANDMARK_PAUSE
                self._transition(State.RECOVERY)
                return

            # Timeout guard — wall or traction issue
            if now - self._phase_t0 > TIMEOUT:
                self._stop()
                self.get_logger().warn(
                    f'MOVE_SEGMENT: timeout after {TIMEOUT:.1f} s '
                    f'(travelled {dist:.2f} m) — skipping to pause')
                self._transition(State.LANDMARK_PAUSE)
                return

            # Distance goal reached
            if dist >= self._seg_dist:
                self._stop()
                self._segments_done += 1
                # Store pose as a revisit candidate
                self._stored_poses.append(self._pose.copy())
                self.get_logger().info(
                    f'MOVE_SEGMENT: complete  '
                    f'dist={dist:.2f} m  segments_done={self._segments_done}  '
                    f'rtab_wm={self._rtab_wm}')
                self._transition(State.LANDMARK_PAUSE)
            else:
                self._drive(self._fwd_speed, 0.0)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_landmark_pause(self, now: float):
        """
        Stop at this landmark, do a brief left look so RTAB-Map accumulates
        frames from a slightly different angle (helps loop closure later), then
        return to heading and decide whether to revisit.

        Sub-phases:
          0: stop, record base yaw
          1: initial pause (pause_dur / 2)
          2: turn left (look left)
          3: pause at left
          4: return to base yaw
          5: final pause (pause_dur / 2) → REVISIT_CHECK
        """
        HALF = max(self._pause_dur / 2.0, 0.5)
        LOOK = 0.8   # s — duration of the left-look pause

        if self._sub_phase == 0:
            self._stop()
            self._look_base_yaw = self._pose.yaw
            self.get_logger().info(
                f'LANDMARK_PAUSE: observing  '
                f'pose={self._pose}  rtab_wm={self._rtab_wm}')
            self._sub_phase = 1

        elif self._sub_phase == 1:   # initial pause
            if now - self._phase_t0 >= HALF:
                self._sub_phase = 2

        elif self._sub_phase == 2:   # look left
            if self._turn_toward_yaw(self._look_base_yaw + self._obs_angle):
                self._phase_t0  = now
                self._sub_phase = 3

        elif self._sub_phase == 3:   # pause at left
            if now - self._phase_t0 >= LOOK:
                self._sub_phase = 4

        elif self._sub_phase == 4:   # return to base yaw
            if self._turn_toward_yaw(self._look_base_yaw):
                self._phase_t0  = now
                self._sub_phase = 5

        elif self._sub_phase == 5:   # final pause
            if now - self._phase_t0 >= HALF:
                self._transition(State.REVISIT_CHECK)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_revisit_check(self, now: float):
        """
        Decide the next action after a landmark pause:

          (a) Trigger a revisit to an earlier pose if:
              - revisit_interval segments have elapsed since the last revisit, OR
              - Spot is more than 3 m from the start area.
          (b) Evaluate map quality if the time budget is nearly exhausted.
          (c) Continue forward with another MOVE_SEGMENT.

        Keeping revisits frequent and returning toward the start area maximises
        the chance that RTAB-Map's local loop closure fires.
        """
        elapsed = now - self._mission_t0

        # Time budget nearly gone — check quality now
        if elapsed >= self._max_dur * 0.90:
            self.get_logger().info('REVISIT_CHECK: near time limit → MAP_QUALITY_CHECK')
            self._transition(State.MAP_QUALITY_CHECK)
            return

        # Revisit trigger: every revisit_interval segments (and we have stored poses)
        interval_trigger = (
            self._segments_done > 0 and
            self._segments_done % self._revisit_ivl == 0 and
            len(self._stored_poses) >= 2
        )

        # Revisit trigger: drifting far from start
        dist_from_start = (self._pose.distance_to(self._start_pose)
                           if self._start_pose else 0.0)
        range_trigger = dist_from_start > 3.0

        if interval_trigger or range_trigger:
            reason = 'interval' if interval_trigger else f'range ({dist_from_start:.1f} m)'
            # Pick the stored pose closest to the start to maximise loop closure odds.
            # Exclude the most-recently stored pose (we just came from there).
            candidates = self._stored_poses[:-1]
            if candidates:
                self._revisit_target = min(
                    candidates,
                    key=lambda p: p.distance_to(self._start_pose)
                )
                self.get_logger().info(
                    f'REVISIT_CHECK: triggering revisit ({reason}) → {self._revisit_target}')
                self._transition(State.REVISIT_PREVIOUS)
                return

        # Default: keep exploring forward
        self._transition(State.MOVE_SEGMENT)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_revisit_previous(self, now: float):
        """
        Navigate toward a previously stored pose to create a loop closure opportunity.
        Uses a turn-then-drive approach with proportional heading correction.
        No Nav2 required — pure odometry-based motion.

        Sub-phases:
          0: log intent
          1: turn to face the target bearing
          2: drive toward target with heading correction; stop when within tolerance
          3: stand still at target for REVISIT_PAUSE seconds
             → MOVE_SEGMENT
        """
        REVISIT_PAUSE   = 3.0    # s — how long to stand at the revisit pose
        REVISIT_TIMEOUT = 30.0   # s — give up if we cannot reach the target
        REALIGN_THRESH  = math.radians(30.0)   # realign heading if error > this

        target = self._revisit_target
        if target is None:
            self._transition(State.MOVE_SEGMENT)
            return

        if self._sub_phase == 0:
            self.get_logger().info(
                f'REVISIT_PREVIOUS: heading to {target}  '
                f'distance={self._pose.distance_to(target):.2f} m')
            self._sub_phase = 1

        elif self._sub_phase == 1:
            # Turn to face the target before driving
            bearing = self._pose.bearing_to(target)
            if self._turn_toward_yaw(bearing, tol_deg=8.0):
                self._sub_phase = 2

        elif self._sub_phase == 2:
            dist = self._pose.distance_to(target)

            if self._obstacle_ahead():
                self._stop()
                self.get_logger().warn(
                    'REVISIT_PREVIOUS: obstacle encountered — abandoning revisit')
                self._revisit_target = None
                self._transition(State.LANDMARK_PAUSE)
                return

            if now - self._phase_t0 > REVISIT_TIMEOUT:
                self._stop()
                self.get_logger().warn('REVISIT_PREVIOUS: timeout — skipping revisit')
                self._revisit_target = None
                self._transition(State.MOVE_SEGMENT)
                return

            if dist <= self._rev_tol:
                self._stop()
                self.get_logger().info(
                    f'REVISIT_PREVIOUS: arrived  dist={dist:.2f} m  '
                    f'loops_so_far={self._rtab_loops}')
                self._phase_t0  = now
                self._sub_phase = 3
            else:
                # Proportional heading correction.  If heading error grows large,
                # stop translating and just turn to re-align.
                bearing  = self._pose.bearing_to(target)
                head_err = wrap_angle(bearing - self._pose.yaw)
                if abs(head_err) > REALIGN_THRESH:
                    # Re-align in place before driving further
                    self._drive(0.0, math.copysign(self._ang_speed, head_err))
                else:
                    # Drive with gentle steering correction
                    angular = 0.4 * head_err
                    self._drive(self._fwd_speed * 0.8, angular)

        elif self._sub_phase == 3:
            # Stand at the revisit pose — prime loop closure opportunity
            if now - self._phase_t0 >= REVISIT_PAUSE:
                self.get_logger().info(
                    f'REVISIT_PREVIOUS: done  '
                    f'loop_closures_total={self._rtab_loops}  '
                    f'rtab_wm={self._rtab_wm}')
                self._revisit_target = None
                self._transition(State.MOVE_SEGMENT)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_map_quality_check(self, now: float):
        """
        Decide whether the map is good enough to save, or whether to continue.

        Heuristics (all configurable via parameters):
          1. Minimum elapsed time (always map for at least 30 s)
          2. RTAB-Map working memory ≥ min_nodes_for_quality
          3. At least min_loop_closures detected (preferred, not hard-required)

        If time and node count are satisfied, save regardless of loop closures
        (log a warning so the operator knows to revisit the start area next time).
        """
        elapsed   = now - self._mission_t0
        nodes_ok  = self._rtab_wm  >= self._min_nodes
        time_ok   = elapsed >= 30.0
        loops_ok  = self._rtab_loops >= self._min_loops

        self.get_logger().info(
            f'MAP_QUALITY_CHECK:\n'
            f'  elapsed={elapsed:.0f} s  (min 30 s: {"✓" if time_ok else "✗"})\n'
            f'  rtab_wm={self._rtab_wm}  (min {self._min_nodes}: {"✓" if nodes_ok else "✗"})\n'
            f'  loop_closures={self._rtab_loops}  (want {self._min_loops}: {"✓" if loops_ok else "✗"})\n'
            f'  segments_done={self._segments_done}')

        if not time_ok:
            self.get_logger().warn(
                'MAP_QUALITY_CHECK: too early — continuing mapping')
            self._transition(State.MOVE_SEGMENT)
            return

        if not nodes_ok:
            self.get_logger().warn(
                f'MAP_QUALITY_CHECK: only {self._rtab_wm}/{self._min_nodes} nodes '
                f'— continuing mapping')
            self._transition(State.MOVE_SEGMENT)
            return

        if not loops_ok:
            self.get_logger().warn(
                f'MAP_QUALITY_CHECK: no loop closures yet '
                f'(desired {self._min_loops}) — saving map anyway. '
                f'Consider re-running mapping and walking Spot back to the start.')

        self.get_logger().info('MAP_QUALITY_CHECK: map quality sufficient → saving')
        self._transition(State.SAVE_MAP)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_save_map(self, now: float):
        """
        Call the RTAB-Map backup service to flush the database to disk
        (/root/.ros/rtabmap.db inside the container).

        Sub-phases:
          0: wait for service readiness, then call it
          1: wait for the async callback (_on_backup_done) to transition us out
        """
        if self._sub_phase == 0:
            self._stop()
            if not self._backup_cli.service_is_ready():
                self._warn_throttled(
                    'SAVE_MAP: waiting for /rtabmap/rtabmap/backup service...')
                if now - self._phase_t0 > 15.0:
                    self.get_logger().error(
                        'SAVE_MAP: backup service unavailable after 15 s — skipping')
                    self._transition(State.SWITCH_TO_LOCALIZATION)
                return

            self.get_logger().info('SAVE_MAP: calling /rtabmap/rtabmap/backup...')
            future = self._backup_cli.call_async(Empty.Request())
            future.add_done_callback(self._on_backup_done)
            self._sub_phase = 1

        elif self._sub_phase == 1:
            # _on_backup_done callback drives the next transition.
            # Guard against a hung service call.
            if now - self._phase_t0 > 30.0:
                self.get_logger().error('SAVE_MAP: backup call timed out')
                self._transition(State.SWITCH_TO_LOCALIZATION)

    def _on_backup_done(self, future):
        """Async callback from the backup service call."""
        try:
            future.result()
            self.get_logger().info('SAVE_MAP: database backup complete ✓')
        except Exception as exc:
            self.get_logger().error(f'SAVE_MAP: backup failed — {exc}')
        self._transition(State.SWITCH_TO_LOCALIZATION)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_switch_to_localization(self, now: float):
        """
        v1: log the next manual steps.
        Future: could write a flag file or send a lifecycle event to a supervisor
        that automatically kills the mapping launch and starts localization.
        """
        elapsed = now - self._mission_t0
        self.get_logger().info(
            '\n'
            '══════════════════════════════════════════════════════════════\n'
            '  Mapping phase complete.\n'
            f'  Elapsed: {elapsed:.0f} s\n'
            f'  Segments driven: {self._segments_done}\n'
            f'  RTAB-Map nodes: {self._rtab_wm}\n'
            f'  Loop closures: {self._rtab_loops}\n'
            '\n'
            '  Next steps:\n'
            '  1. Copy map to host:\n'
            '       docker cp spot_nav:/root/.ros/rtabmap.db \\\n'
            '         /Users/temp/Desktop/spot_nav_repo/rtabmap.db\n'
            '\n'
            '  2. Kill RTAB-Map (T3) and restart in localization mode:\n'
            '       bash /root/start_rtabmap_loc.sh\n'
            '\n'
            '  3. Start Nav2 (T6):\n'
            '       ros2 launch nav2_bringup navigation_launch.py \\\n'
            '         params_file:=/root/nav2_params.yaml use_sim_time:=false\n'
            '══════════════════════════════════════════════════════════════')
        self._transition(State.DONE)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_done(self, now: float):
        """Terminal state — stop the robot and idle."""
        if self._sub_phase == 0:
            self._stop()
            self.get_logger().info('DONE: autonomous_mapper finished cleanly.')
            self._sub_phase = 1
        # Stay in DONE; kill the node externally (Ctrl+C)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_recovery(self, now: float):
        """
        Respond to an obstacle or unexpected condition by backing up and turning
        away, then returning to the state specified by _recovery_return.

        Sub-phases:
          0: stop, record backup start pose
          1: back up BACKUP_DIST metres
          2: turn TURN_ANGLE degrees away
          3: return to _recovery_return state
        """
        BACKUP_DIST  = 0.30              # m to reverse
        BACKUP_SPEED = 0.10              # m/s (slow for safety)
        TURN_ANGLE   = math.radians(60.0)

        if self._sub_phase == 0:
            self.get_logger().warn(
                f'RECOVERY: reason="{self._recovery_reason}"  '
                f'will return to {self._recovery_return.name}')
            self._stop()
            self._move_start = self._pose.copy()
            self._sub_phase  = 1

        elif self._sub_phase == 1:
            dist = self._pose.distance_to(self._move_start)
            if dist < BACKUP_DIST:
                self._drive(-BACKUP_SPEED, 0.0)
            else:
                self._stop()
                # Turn left by default; could be made adaptive in a future version
                self._recovery_turn_yaw = self._pose.yaw + TURN_ANGLE
                self._sub_phase = 2

        elif self._sub_phase == 2:
            if self._turn_toward_yaw(self._recovery_turn_yaw):
                self.get_logger().info(
                    f'RECOVERY: complete → {self._recovery_return.name}')
                self._transition(self._recovery_return)

    # ──────────────────────────────────────────────────────────────────────────

    def _do_abort(self, now: float):
        """Terminal error state — stop and log."""
        if self._sub_phase == 0:
            self._stop()
            self.get_logger().error(
                'ABORT: autonomous_mapper stopped due to a fatal error. '
                'Check that Spot driver, tf_keep_alive, and RTAB-Map are running.')
            self._sub_phase = 1

    # ── main tick ─────────────────────────────────────────────────────────────

    _ACTIVE_STATES = {
        State.STARTUP, State.VERIFY_SYSTEM, State.START_MAPPING,
        State.INITIAL_OBSERVE, State.MOVE_SEGMENT, State.LANDMARK_PAUSE,
        State.REVISIT_CHECK, State.REVISIT_PREVIOUS, State.RECOVERY,
    }

    def _tick(self):
        now = self._now()

        # ── global time budget ────────────────────────────────────────────────
        # Force transition to MAP_QUALITY_CHECK once the budget runs out.
        # Only applies while in active (non-terminal, non-save) states.
        if (self._mission_t0 > 0 and
                self._state in self._ACTIVE_STATES and
                now - self._mission_t0 >= self._max_dur):
            self.get_logger().info(
                f'Time budget ({self._max_dur:.0f} s) elapsed → MAP_QUALITY_CHECK')
            self._transition(State.MAP_QUALITY_CHECK)
            return

        # ── state dispatch ────────────────────────────────────────────────────
        handlers = {
            State.STARTUP:                self._do_startup,
            State.VERIFY_SYSTEM:          self._do_verify_system,
            State.START_MAPPING:          self._do_start_mapping,
            State.INITIAL_OBSERVE:        self._do_initial_observe,
            State.MOVE_SEGMENT:           self._do_move_segment,
            State.LANDMARK_PAUSE:         self._do_landmark_pause,
            State.REVISIT_CHECK:          self._do_revisit_check,
            State.REVISIT_PREVIOUS:       self._do_revisit_previous,
            State.MAP_QUALITY_CHECK:      self._do_map_quality_check,
            State.SAVE_MAP:               self._do_save_map,
            State.SWITCH_TO_LOCALIZATION: self._do_switch_to_localization,
            State.DONE:                   self._do_done,
            State.RECOVERY:               self._do_recovery,
            State.ABORT:                  self._do_abort,
        }
        handler = handlers.get(self._state)
        if handler:
            handler(now)

    # ── shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Always stop the robot on shutdown."""
        self._stop()
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = AutonomousMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
