#!/usr/bin/env python3

import math
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    # Roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # Yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class AdaptivePlanner(Node):
    def __init__(self):
        super().__init__('adaptive_planner')

        # --- Parameters (kept yours, added a few) ---
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('angle_tolerance', 0.1)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('replan_distance', 0.5)

        # New: planner & safety knobs
        self.declare_parameter('path_frame_id', 'odom')     # for RViz markers/path
        self.declare_parameter('treat_unknown_as_free', False)
        self.declare_parameter('straight_check_horizon_m', 1.5)
        self.declare_parameter('los_hysteresis', 3)

        # Edge-follow chooser/tuning
        self.declare_parameter('edge_ring_radius_cells', 3)
        self.declare_parameter('edge_ring_step', 16)
        self.declare_parameter('edge_side_lock_secs', 2.0)
        self.declare_parameter('edge_stuck_cycles', 12)
        self.declare_parameter('edge_worse_ratio', 1.08)

        # NEW: inflation-aware edge detection & multi-radius growth
        self.declare_parameter('edge_neighbor_min_cost', 1)       # treat any cost>=1 as obstacle-like for picking edges
        self.declare_parameter('edge_max_ring_radius_cells', 10)  # grow search out to this many cells if needed

        # pull values
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.obstacle_threshold = int(self.get_parameter('obstacle_threshold').value)
        self.replan_distance = float(self.get_parameter('replan_distance').value)
        self.path_frame_id = self.get_parameter('path_frame_id').value

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Navigation state
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.is_navigating = False

        # Modes
        self.mode = 'STRAIGHT'  # 'STRAIGHT' or 'EDGE_FOLLOW'
        self._los_clear_count = 0
        self.edge_side = None  # 'left' or 'right'
        self.edge_last_pick_time = 0.0
        self.edge_no_candidate_count = 0
        self.edge_best_goal_dist = float('inf')
        self._edge_target = None  # (wx, wy, score)

        # Costmap
        self.costmap = None
        self.costmap_resolution = 0.05
        self.costmap_origin_x = 0.0
        self.costmap_origin_y = 0.0
        self.costmap_width = 0
        self.costmap_height = 0
        self.costmap_frame_id = 'map'

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/navigation_markers', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/costmap', self.costmap_callback, 10
        )

        # Control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Adaptive Planner (straight + edge-follow) initialized')

    # --- Callbacks ---

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin_x = msg.info.origin.position.x
        self.costmap_origin_y = msg.info.origin.position.y
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap_frame_id = msg.header.frame_id

        if not hasattr(self, '_costmap_logged'):
            self._costmap_logged = True
            occupied = int(np.sum(self.costmap >= self.obstacle_threshold))
            total = int(self.costmap_width * self.costmap_height)
            self.get_logger().info(
                f'Costmap {self.costmap_width}x{self.costmap_height} res={self.costmap_resolution:.3f}m '
                f'origin=({self.costmap_origin_x:.2f},{self.costmap_origin_y:.2f}) '
                f'occupied={occupied}/{total} ({100*occupied/total:.1f}%)'
            )

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.goal_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.is_navigating = True
        self.mode = 'STRAIGHT'
        self._los_clear_count = 0
        self.edge_side = None
        self._edge_target = None
        self.get_logger().info(f'New goal: ({self.goal_x:.2f},{self.goal_y:.2f}), yaw={math.degrees(self.goal_yaw):.1f}°')

    # --- Grid helpers ---

    def world_to_map(self, x, y):
        mx = math.floor((x - self.costmap_origin_x) / self.costmap_resolution)
        my = math.floor((y - self.costmap_origin_y) / self.costmap_resolution)
        return int(mx), int(my)

    def map_to_world(self, mx, my):
        x = (mx + 0.5) * self.costmap_resolution + self.costmap_origin_x
        y = (my + 0.5) * self.costmap_resolution + self.costmap_origin_y
        return x, y

    def cost_at(self, mx, my):
        if mx < 0 or mx >= self.costmap_width or my < 0 or my >= self.costmap_height:
            return 100  # outside treated as blocked
        return int(self.costmap[my, mx])

    def is_free_cell(self, mx, my):
        c = self.cost_at(mx, my)
        if c == -1 and not self.get_parameter('treat_unknown_as_free').value:
            return False
        return 0 <= c < self.obstacle_threshold

    def is_obstacle_cell(self, mx, my):
        c = self.cost_at(mx, my)
        return (c == -1 and not self.get_parameter('treat_unknown_as_free').value) or (c >= self.obstacle_threshold)

    def neighbor8(self):
        return [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]

    # PATCHED: inflation-aware edge detection
    def cell_has_obstacle_neighbor(self, mx, my):
        """Edge detection: treat neighbors with cost >= edge_neighbor_min_cost as 'obstacle-like'."""
        min_cost = int(self.get_parameter('edge_neighbor_min_cost').value)
        for dx, dy in self.neighbor8():
            nx, ny = mx + dx, my + dy
            c = self.cost_at(nx, ny)
            # unknown is still treated as blocked unless treat_unknown_as_free=True
            if c == -1 and not self.get_parameter('treat_unknown_as_free').value:
                return True
            if c >= min_cost:
                return True
        return False

    # --- Validity & line checks ---

    def is_point_valid(self, x, y):
        if self.costmap is None:
            return True
        mx, my = self.world_to_map(x, y)
        return self.is_free_cell(mx, my)

    def is_line_free(self, x1, y1, x2, y2):
        if self.costmap is None:
            return True
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 1e-6:
            return self.is_point_valid(x1, y1)
        step = max(self.costmap_resolution * 0.5, 0.02)
        steps = max(1, int(dist / step))
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not self.is_point_valid(x, y):
                return False
        return True

    def has_line_of_sight_to_goal(self):
        if self.goal_x is None:
            return False
        return self.is_line_free(self.current_x, self.current_y, self.goal_x, self.goal_y)

    def straight_ray_blocked_ahead(self):
        if self.costmap is None or self.goal_x is None:
            return False
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist = math.hypot(dx, dy)
        horizon = min(dist, float(self.get_parameter('straight_check_horizon_m').value))
        if horizon <= 0.05:
            return False
        # sample along horizon
        step = max(self.costmap_resolution * 0.5, 0.02)
        steps = max(1, int(horizon / step))
        for i in range(1, steps + 1):
            t = (i / steps) * (horizon / dist)
            x = self.current_x + t * dx
            y = self.current_y + t * dy
            if not self.is_point_valid(x, y):
                return True
        return False

    # --- Edge-follow target selection (PATCHED: multi-radius growth) ---

    def pick_edge_follow_target(self, side):
        """
        side: 'left' or 'right'
        Return (wx, wy, score) or None.
        Grows the search ring until it finds the inflated band.
        """
        if self.costmap is None:
            return None

        base_r = max(1, int(self.get_parameter('edge_ring_radius_cells').value))
        max_r  = max(base_r, int(self.get_parameter('edge_max_ring_radius_cells').value))
        samples = max(12, int(self.get_parameter('edge_ring_step').value))

        rx_mx, rx_my = self.world_to_map(self.current_x, self.current_y)
        if self.goal_x is not None:
            g_ang = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        else:
            g_ang = self.current_yaw

        side_bias = (math.pi / 2) if side == 'left' else (-math.pi / 2)
        best = None  # (score, wx, wy)

        # Grow outward until we touch the inflated band or run out of radius
        for ring_r in range(base_r, max_r + 1):
            found_any = False
            for k in range(samples):
                # concentrate samples around the tangential direction on the chosen side
                ang = g_ang + side_bias + (2 * math.pi) * (k / samples - 0.5) * 0.35
                mx = rx_mx + int(round(ring_r * math.cos(ang)))
                my = rx_my + int(round(ring_r * math.sin(ang)))
                if not self.is_free_cell(mx, my):
                    continue
                # edge = free cell that is adjacent to any positive-cost (or unknown) neighbor
                if not self.cell_has_obstacle_neighbor(mx, my):
                    continue

                wx, wy = self.map_to_world(mx, my)
                g = math.hypot(self.goal_x - wx, self.goal_y - wy) if self.goal_x is not None else 0.0
                hd = abs(self.normalize_angle(math.atan2(wy - self.current_y, wx - self.current_x) - self.current_yaw))
                score = g + 0.2 * hd
                candidate = (score, wx, wy)
                if best is None or candidate[0] < best[0]:
                    best = candidate
                found_any = True

            # As soon as we found candidates on this ring, stop growing further
            if found_any:
                break

        if best is None:
            return None
        return (best[1], best[2], best[0])

    def choose_edge_side(self):
        left = self.pick_edge_follow_target('left')
        right = self.pick_edge_follow_target('right')
        if left is None and right is None:
            return None, None
        if left is None:
            return 'right', right
        if right is None:
            return 'left', left
        return ('left', left) if left[2] <= right[2] else ('right', right)

    # --- Control loop & executors ---

    def control_loop(self):
        if not self.is_navigating or self.goal_x is None:
            return

        # mode transitions
        if self.mode == 'STRAIGHT':
            if self.straight_ray_blocked_ahead():
                self.stop_robot()
                self.mode = 'EDGE_FOLLOW'
                self.edge_side, cand = self.choose_edge_side()
                self.edge_last_pick_time = self.get_clock().now().nanoseconds / 1e9
                self.edge_no_candidate_count = 0
                self.edge_best_goal_dist = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
                if cand is None:
                    self.get_logger().warn('EDGE_FOLLOW: no initial candidate on either side')
                    self._edge_target = None
                else:
                    self._edge_target = cand  # (wx, wy, score)
                    self.get_logger().info(f'EDGE_FOLLOW: chose {self.edge_side}-hand first')

        if self.mode == 'EDGE_FOLLOW':
            # LOS → back to straight (with hysteresis)
            if self.has_line_of_sight_to_goal():
                self._los_clear_count += 1
            else:
                self._los_clear_count = 0
            if self._los_clear_count >= int(self.get_parameter('los_hysteresis').value):
                self.mode = 'STRAIGHT'
                self.edge_side = None
                self._edge_target = None
                self.get_logger().info('Line of sight restored → STRAIGHT')

        # act by mode
        if self.mode == 'STRAIGHT':
            self.follow_straight_to_goal()
        else:
            self.edge_follow_step()

        # publish simple viz
        self.publish_viz()

    def follow_straight_to_goal(self):
        # goal reached?
        dist_to_goal = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        if dist_to_goal < self.goal_tolerance:
            # optional yaw align
            if self.goal_yaw is not None:
                ang_err = self.normalize_angle(self.goal_yaw - self.current_yaw)
                if abs(ang_err) > self.angle_tolerance:
                    cmd = Twist()
                    cmd.angular.z = math.copysign(self.angular_speed, ang_err)
                    self.cmd_vel_pub.publish(cmd)
                    return
            self.stop_robot()
            self.is_navigating = False
            self.get_logger().info('🎉 Goal reached!')
            return

        # direct go-to
        ang = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        ang_err = self.normalize_angle(ang - self.current_yaw)

        cmd = Twist()
        # turn-in-place if large heading error
        if abs(ang_err) > 0.4:
            cmd.angular.z = math.copysign(self.angular_speed, ang_err)
            cmd.linear.x = 0.0
        else:
            k_ang = 1.0
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, k_ang * ang_err))
            cmd.linear.x = min(self.linear_speed, 0.8 * dist_to_goal) * max(0.0, math.cos(ang_err))
        self.cmd_vel_pub.publish(cmd)

    def edge_follow_step(self):
        # update/choose target on current side
        cand = self.pick_edge_follow_target(self.edge_side) if self.edge_side else None
        if cand is None:
            self.edge_no_candidate_count += 1
        else:
            self.edge_no_candidate_count = 0
            self._edge_target = cand

        # progress check
        cur_dist = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        made_progress = cur_dist < self.edge_best_goal_dist - 0.02  # 2 cm margin
        if made_progress:
            self.edge_best_goal_dist = cur_dist

        # consider flipping side if stuck/worse
        now_s = self.get_clock().now().nanoseconds / 1e9
        lock_secs = float(self.get_parameter('edge_side_lock_secs').value)
        worse_ratio = float(self.get_parameter('edge_worse_ratio').value)
        stuck_cycles = int(self.get_parameter('edge_stuck_cycles').value)

        should_consider_flip = (
            (now_s - self.edge_last_pick_time > lock_secs) and
            (self.edge_no_candidate_count >= stuck_cycles or not made_progress)
        )

        if should_consider_flip:
            other = 'left' if self.edge_side == 'right' else 'right'
            other_cand = self.pick_edge_follow_target(other)
            if other_cand is not None:
                cur_score = self._edge_target[2] if (self._edge_target is not None) else float('inf')
                if other_cand[2] * worse_ratio < cur_score:
                    self.edge_side = other
                    self._edge_target = other_cand
                    self.edge_last_pick_time = now_s
                    self.edge_no_candidate_count = 0
                    self.get_logger().info(f'EDGE_FOLLOW: flipping to {self.edge_side}-hand')

        # execute edge step
        if self._edge_target is not None:
            self.follow_edge_to_target(self._edge_target[0], self._edge_target[1])
        else:
            self.stop_robot()
            self.get_logger().warn('EDGE_FOLLOW: no edge target — holding')

    def follow_edge_to_target(self, wx, wy):
        dx = wx - self.current_x
        dy = wy - self.current_y
        dist = math.hypot(dx, dy)
        ang = math.atan2(dy, dx)
        ang_err = self.normalize_angle(ang - self.current_yaw)

        cmd = Twist()
        if abs(ang_err) > 0.6:
            cmd.angular.z = math.copysign(self.angular_speed, ang_err)
            cmd.linear.x = 0.0
        else:
            k_ang = 1.0
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, k_ang * ang_err))
            step_speed = 0.5 * self.linear_speed
            cmd.linear.x = min(step_speed, 0.6 * dist) * max(0.0, math.cos(ang_err))
        self.cmd_vel_pub.publish(cmd)

    # --- Viz ---

    def publish_viz(self):
        # Path: simple two-point line to goal in STRAIGHT; to edge target in EDGE_FOLLOW
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.path_frame_id

        def add_pose(x, y):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.path_frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        add_pose(self.current_x, self.current_y)
        if self.mode == 'STRAIGHT' and self.goal_x is not None:
            add_pose(self.goal_x, self.goal_y)
        elif self.mode == 'EDGE_FOLLOW' and self._edge_target is not None:
            add_pose(self._edge_target[0], self._edge_target[1])
        self.path_pub.publish(path_msg)

        # Markers: current target (color by mode) + goal arrow
        ma = MarkerArray()

        # Target marker
        marker = Marker()
        marker.header.frame_id = self.path_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        if self.mode == 'STRAIGHT' and self.goal_x is not None:
            marker.pose.position.x = self.goal_x
            marker.pose.position.y = self.goal_y
        elif self.mode == 'EDGE_FOLLOW' and self._edge_target is not None:
            marker.pose.position.x = self._edge_target[0]
            marker.pose.position.y = self._edge_target[1]
        else:
            marker.pose.position.x = self.current_x
            marker.pose.position.y = self.current_y
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25
        if self.mode == 'STRAIGHT':
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0  # red
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.4, 1.0, 1.0  # blue-ish
        ma.markers.append(marker)

        # Goal arrow
        if self.goal_x is not None:
            gmk = Marker()
            gmk.header.frame_id = self.path_frame_id
            gmk.header.stamp = self.get_clock().now().to_msg()
            gmk.ns = 'goal'
            gmk.id = 1
            gmk.type = Marker.ARROW
            gmk.action = Marker.ADD
            gmk.pose.position.x = self.goal_x
            gmk.pose.position.y = self.goal_y
            gmk.scale.x = 0.3
            gmk.scale.y = 0.08
            gmk.scale.z = 0.08
            gmk.color.r, gmk.color.g, gmk.color.b, gmk.color.a = 0.0, 1.0, 0.0, 1.0
            ma.markers.append(gmk)

        self.markers_pub.publish(ma)

    # --- Utils ---

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
