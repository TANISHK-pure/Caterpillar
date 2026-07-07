#!/usr/bin/env python3
"""
hybrid_astar_node.py  --  ROS 2 node: Hybrid A* planner + pure-pursuit executor.

Subscribes
----------
/truck_1/odom          nav_msgs/Odometry        live pose from Gazebo bridge
/truck_1/goal          geometry_msgs/PoseStamped  planning target
/truck_1/cancel        std_msgs/Empty             abort & stop

Publishes
----------
/truck_1/cmd_vel       geometry_msgs/Twist        velocity commands to Gazebo
/truck_1/planned_path  nav_msgs/Path              planned path  (visualise in RViz)
/truck_1/lookahead_pt  geometry_msgs/PointStamped lookahead marker (RViz)
/truck_1/plan_status   std_msgs/String            IDLE/PLANNING/EXECUTING/DONE/FAILED

Quick-start
-----------
# Terminal 1 — launch Gazebo
ros2 launch cat_paddock_sim paddock_demo.launch.py

# Terminal 2 — run the planner node
ros2 run cat_paddock_sim hybrid_astar_node

# Terminal 3 — send a goal (truck_1 drives to dump approach)
ros2 topic pub --once /truck_1/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"},
    pose: {position: {x: 50.0, y: 110.0, z: 0.0},
           orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}'

# Terminal 4 — watch status
ros2 topic echo /truck_1/plan_status

Parameters (override with --ros-args -p name:=value)
-----------------------------------------------------
odom_topic      /truck_1/odom
cmd_vel_topic   /truck_1/cmd_vel
goal_topic      /truck_1/goal
frame_id        world
publish_rate    20.0     Hz
lookahead_min   4.0      m   tight enough to track curves on a 9m truck
lookahead_max   10.0     m   scales with speed on straights
max_linear      2.0      m/s top forward speed
max_angular     2.5      rad/s angular cap — matches teleop ANGULAR_SPEED
goal_pos_tol    3.0      m   position goal tolerance
goal_th_tol     0.35     rad heading goal tolerance (~20°)
"""

import math
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node as RosNode
from geometry_msgs.msg import (Point, PointStamped, PoseStamped,
                                Quaternion, Twist)
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, String


# local import (same Python package)
from cat_paddock_sim.hybrid_astar_core import plan as _hastar_plan, _wrap


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ---------------------------------------------------------------------------
# Status strings
# ---------------------------------------------------------------------------
IDLE      = 'IDLE'
PLANNING  = 'PLANNING'
EXECUTING = 'EXECUTING'
DONE      = 'DONE'
FAILED    = 'FAILED'


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class HybridAstarNode(RosNode):

    def __init__(self):
        super().__init__('hybrid_astar_node')

        # ---- declare parameters ----
        self.declare_parameter('odom_topic',    '/truck_1/odom')
        self.declare_parameter('cmd_vel_topic', '/truck_1/cmd_vel')
        self.declare_parameter('goal_topic',    '/truck_1/goal')
        self.declare_parameter('frame_id',      'world')
        self.declare_parameter('publish_rate',  20.0)
        self.declare_parameter('lookahead_min',  4.0)   # FIX: was 8.0 — too large for tight turns
        self.declare_parameter('lookahead_max', 10.0)   # FIX: was 15.0
        self.declare_parameter('max_linear',     2.0)   # m/s
        self.declare_parameter('max_angular',    2.5)   # FIX: was 1.0 — matches ANGULAR_SPEED in teleop
        self.declare_parameter('goal_pos_tol',   3.0)
        self.declare_parameter('goal_th_tol',    0.35)

        def gp(name):
            return self.get_parameter(name).value

        self._odom_topic = gp('odom_topic')
        self._cmd_topic  = gp('cmd_vel_topic')
        self._frame      = gp('frame_id')
        self._rate       = float(gp('publish_rate'))
        self._lk_min     = float(gp('lookahead_min'))
        self._lk_max     = float(gp('lookahead_max'))
        self._max_lin    = float(gp('max_linear'))
        self._max_ang    = float(gp('max_angular'))
        self._pos_tol    = float(gp('goal_pos_tol'))
        self._th_tol     = float(gp('goal_th_tol'))

        # ---- spawn pose (world frame) — matches paddock_demo.launch.py ----
        self.declare_parameter('spawn_x',   -30.0)
        self.declare_parameter('spawn_y',   -50.0)
        self.declare_parameter('spawn_yaw',  0.0)   # facing +X (no -yaw in launch)
        self._spawn_x   = float(self.get_parameter('spawn_x').value)
        self._spawn_y   = float(self.get_parameter('spawn_y').value)
        self._spawn_th  = float(self.get_parameter('spawn_yaw').value)

        # ---- mutable state (protected by _lock) ----
        self._lock   = threading.Lock()
        self._odom_received: bool = False
        self._pose:   Optional[Tuple[float, float, float]] = None  # (x,y,yaw)
        self._goal:   Optional[Tuple[float, float, float]] = None  # (x,y,yaw)
        self._path:   List[Tuple[float, float, float]] = []
        self._wp_idx: int   = 0
        self._status: str   = IDLE
        self._speed:  float = 0.0   # last commanded linear speed (for adaptive lookahead)

        # ---- publishers ----
        self._pub_cmd    = self.create_publisher(Twist,        gp('cmd_vel_topic'),       10)
        self._pub_path   = self.create_publisher(Path,         '/truck_1/planned_path',   10)
        self._pub_look   = self.create_publisher(PointStamped, '/truck_1/lookahead_pt',   10)
        self._pub_status = self.create_publisher(String,       '/truck_1/plan_status',    10)

        # ---- subscribers ----
        self.create_subscription(Odometry,    gp('odom_topic'), self._cb_odom,   10)
        self.create_subscription(PoseStamped, gp('goal_topic'), self._cb_goal,   10)
        self.create_subscription(Empty, '/truck_1/cancel',      self._cb_cancel, 10)

        # ---- control timer ----
        self.create_timer(1.0 / self._rate, self._control_loop)

        self.get_logger().info(
            f"[hybrid_astar] ready  "
            f"odom={self._odom_topic}  cmd={self._cmd_topic}  "
            f"max_ang={self._max_ang:.1f} rad/s  lk_min={self._lk_min:.1f}m")

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _cb_odom(self, msg: Odometry):
        # Diff-drive odom is RELATIVE to spawn point (starts at 0,0,0).
        # World pose = spawn_pose + odom displacement.
        dx   = msg.pose.pose.position.x
        dy   = msg.pose.pose.position.y
        dyaw = _quat_to_yaw(msg.pose.pose.orientation)
        c = math.cos(self._spawn_th); s = math.sin(self._spawn_th)
        wx   = self._spawn_x + c * dx - s * dy
        wy   = self._spawn_y + s * dx + c * dy
        wyaw = _wrap(self._spawn_th + dyaw)
        with self._lock:
            self._pose = (wx, wy, wyaw)
            self._odom_received = True

    def _cb_goal(self, msg: PoseStamped):
        gx  = msg.pose.position.x
        gy  = msg.pose.position.y
        gth = _quat_to_yaw(msg.pose.orientation)
        self.get_logger().info(
            f"[hybrid_astar] new goal  "
            f"({gx:.1f}, {gy:.1f}, {math.degrees(gth):.0f}°)")
        with self._lock:
            self._goal   = (gx, gy, gth)
            self._path   = []
            self._wp_idx = 0
            self._status = PLANNING
        self._stop()
        threading.Thread(target=self._run_planner, daemon=True).start()

    def _cb_cancel(self, _msg: Empty):
        self.get_logger().warn("[hybrid_astar] cancelled.")
        with self._lock:
            self._path   = []
            self._status = IDLE
        self._stop()

    # ------------------------------------------------------------------
    # Planner  (background thread)
    # ------------------------------------------------------------------
    def _run_planner(self):
        deadline = time.time() + 30.0
        last_log = 0.0
        while time.time() < deadline:
            with self._lock:
                pose   = self._pose
                status = self._status
            if status != PLANNING:
                return

            with self._lock:
                odom_ok = self._odom_received
                pose    = self._pose
            if odom_ok and pose is not None:
                px, py, pth = pose
                self.get_logger().info(
                    f"[hybrid_astar] odom ready — world pose: "
                    f"({px:.2f}, {py:.2f}, {math.degrees(pth):.1f}°)")
                break
            if time.time() - last_log > 2.0:
                self.get_logger().info(
                    "[hybrid_astar] waiting for odom from Gazebo bridge ...")
                last_log = time.time()
            time.sleep(0.2)
        else:
            self.get_logger().error(
                "[hybrid_astar] 30 s timeout — no valid odom.")
            with self._lock:
                self._status = FAILED
            return

        with self._lock:
            pose = self._pose
            goal = self._goal
            if self._status != PLANNING:
                return

        if pose is None or goal is None:
            self.get_logger().error("[hybrid_astar] no pose or goal — cannot plan.")
            with self._lock:
                self._status = FAILED
            return

        sx, sy, sth = pose
        gx, gy, gth = goal

        self.get_logger().info(
            f"[hybrid_astar] planning  "
            f"({sx:.1f},{sy:.1f},{math.degrees(sth):.0f}°) → "
            f"({gx:.1f},{gy:.1f},{math.degrees(gth):.0f}°) ...")

        t0   = time.time()
        path = _hastar_plan(sx, sy, sth, gx, gy, gth,
                            goal_pos_tol=self._pos_tol,
                            goal_th_tol=self._th_tol,
                            max_iter=25_000)
        dt = time.time() - t0

        if path is None:
            self.get_logger().error(
                f"[hybrid_astar] no path found in {dt:.1f}s.")
            with self._lock:
                self._status = FAILED
            return

        self.get_logger().info(
            f"[hybrid_astar] path found: {len(path)} wp in {dt:.1f}s.")
        with self._lock:
            self._path   = path
            self._wp_idx = 0
            self._status = EXECUTING

        self._publish_path(path)

    # ------------------------------------------------------------------
    # Control loop  (pure-pursuit, 20 Hz)
    # ------------------------------------------------------------------
    def _control_loop(self):
        s = String()
        with self._lock:
            s.data = self._status
        self._pub_status.publish(s)

        with self._lock:
            status  = self._status
            path    = list(self._path)
            wp_idx  = self._wp_idx
            pose    = self._pose
            goal    = self._goal

        if status != EXECUTING or not path or pose is None:
            return

        x, y, th = pose
        gx, gy, gth = goal if goal else (0.0, 0.0, 0.0)

        # ---- final goal reached? ----
        if math.hypot(x - gx, y - gy) < self._pos_tol:
            self.get_logger().info("[hybrid_astar] goal reached!")
            self._stop()
            with self._lock:
                self._status = DONE
            return

        # ---- advance wp index past points we have already passed ----
        # Use a smaller threshold (half the minimum lookahead) so we don't
        # skip waypoints too aggressively on tight turns.
        with self._lock:
            advance_thresh = self._lk_min * 0.5
            while (self._wp_idx < len(path) - 1 and
                   math.hypot(x - path[self._wp_idx][0],
                               y - path[self._wp_idx][1]) < advance_thresh):
                self._wp_idx += 1
            wp_idx = self._wp_idx

        # ---- adaptive lookahead: smaller when turning, larger on straights ----
        with self._lock:
            spd = abs(self._speed)

        # Look at the heading of the next few waypoints to detect upcoming turns.
        # Shrink lookahead distance before a sharp corner so we don't overshoot.
        look_ahead_wps = path[wp_idx: wp_idx + 4]
        max_turn = 0.0
        for i in range(len(look_ahead_wps) - 1):
            dth = abs(_wrap(look_ahead_wps[i+1][2] - look_ahead_wps[i][2]))
            max_turn = max(max_turn, dth)

        # Shrink lookahead if a sharp turn is coming up
        turn_factor = max(0.4, 1.0 - max_turn / math.radians(60))
        lookahead = min(self._lk_max,
                        max(self._lk_min,
                            self._lk_min + spd * 1.0)) * turn_factor

        lx, ly, lth = self._lookahead_point(path, wp_idx, x, y, lookahead)
        self._publish_lookahead(lx, ly)

        # ---- pure-pursuit geometry ----
        dx = lx - x
        dy = ly - y

        # Direction to the lookahead point
        target_th = math.atan2(dy, dx)

        # Decide forward vs reverse: pick whichever requires less turning
        err_fwd = abs(_wrap(target_th - th))
        err_rev = abs(_wrap(target_th + math.pi - th))
        reverse = err_rev < err_fwd

        steer_th = _wrap((target_th + math.pi) - th) if reverse else _wrap(target_th - th)

        cmd = Twist()

        # ----------------------------------------------------------------
        # Three-zone controller
        # ----------------------------------------------------------------
        # Zone 1 — LARGE error (>60°): stop and spin in place.
        #   Use full angular speed so the truck actually completes the turn.
        # Zone 2 — MEDIUM error (20°–60°): slow crawl + strong turn.
        # Zone 3 — SMALL error (<20°): normal cruise speed.
        # ----------------------------------------------------------------

        ANG_FULL   = self._max_ang          # 2.5 rad/s — full spin speed
        ANG_MEDIUM = self._max_ang * 0.75   # 1.875 rad/s
        ANG_SMALL  = self._max_ang * 0.5    # 1.25 rad/s — still fast enough to hold a curve

        if abs(steer_th) > math.radians(60):
            # Spin in place — no forward motion at all
            cmd.linear.x  = 0.0
            cmd.angular.z = math.copysign(ANG_FULL, steer_th)

        elif abs(steer_th) > math.radians(20):
            # Crawl forward (or back) while turning firmly
            heading_factor = max(0.2, 1.0 - abs(steer_th) / math.radians(90))
            speed = self._max_lin * 0.35 * heading_factor
            cmd.linear.x  = -speed if reverse else speed
            cmd.angular.z = math.copysign(
                min(ANG_MEDIUM, ANG_MEDIUM * abs(steer_th) / math.radians(60)),
                steer_th)

        else:
            # Normal cruise — proportional steering
            heading_factor = max(0.5, 1.0 - abs(steer_th) / math.radians(20))
            speed = min(self._max_lin,
                        max(0.6, self._max_lin * heading_factor))
            cmd.linear.x  = -speed if reverse else speed
            cmd.angular.z = max(-ANG_SMALL,
                                min(ANG_SMALL, 1.5 * steer_th))

        with self._lock:
            self._speed = cmd.linear.x

        self._pub_cmd.publish(cmd)

    # ------------------------------------------------------------------
    # Pure-pursuit lookahead helper
    # ------------------------------------------------------------------
    def _lookahead_point(self,
                         path: List[Tuple[float, float, float]],
                         wp_idx: int,
                         cx: float, cy: float,
                         lookahead: float
                         ) -> Tuple[float, float, float]:
        """Walk path forward from wp_idx until first point >= lookahead away."""
        for i in range(wp_idx, len(path)):
            px, py, pth = path[i]
            if math.hypot(px - cx, py - cy) >= lookahead:
                return (px, py, pth)
        return path[-1]

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def _publish_path(self, path: List[Tuple[float, float, float]]):
        msg   = Path()
        stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.header.stamp    = stamp
        for x, y, th in path:
            ps = PoseStamped()
            ps.header.frame_id  = self._frame
            ps.header.stamp     = stamp
            ps.pose.position.x  = x
            ps.pose.position.y  = y
            ps.pose.orientation = _yaw_to_quat(th)
            msg.poses.append(ps)
        self._pub_path.publish(msg)

    def _publish_lookahead(self, lx: float, ly: float):
        pt = PointStamped()
        pt.header.frame_id = self._frame
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.point.x = lx
        pt.point.y = ly
        pt.point.z = 1.0
        self._pub_look.publish(pt)

    def _stop(self):
        self._pub_cmd.publish(Twist())


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HybridAstarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()