#!/usr/bin/env python3
import os
import csv
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory

from .pure_pursuit_controller import PurePursuitCore
from .euler_transform import quaternion_to_euler

Point2 = Tuple[float, float]

class AckermannControllerNode(Node):
    def __init__(self):
        super().__init__("ackermann_controller_node")

        self.global_frame = "map"
        self.base_frame = "base_link"
        self.wheelbase = 0.25
        self.lookahead = 0.9
        self.N = 50
        self.window_index = 0
        self._latest_odom = None  # type: Optional[Odometry]
        self.last_odom_time = None  # type: Optional[rclpy.time.Time]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)

        self.controller_PP = PurePursuitCore(self.wheelbase, self.lookahead)

        pkg_share = get_package_share_directory("autonomous_driving_pkg")
        csv_path = os.path.join(pkg_share, "waypointts", "waypoints_xy_cleaned.csv")

        self.route_xy = self._load_waypoints_csv(csv_path)
        self.get_logger().info("Loaded %d waypoints from %s" % (len(self.route_xy), csv_path))

        self.timer = self.create_timer(0.02, self.control_tick)

    def _load_waypoints_csv(self, csv_path):
        waypoints = []
        with open(csv_path, "r", newline="") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                except (ValueError, IndexError):
                    continue
                waypoints.append((x, y))
        return waypoints

    def on_odom(self, msg):
        self._latest_odom = msg
        self.last_odom_time = self.get_clock().now()

    def _select_window(self, car_xy_map):
        cx, cy = car_xy_map
        if not self.route_xy:
            return []

        i1 = min(len(self.route_xy) - 1, self.window_index + self.N)

        best_i = self.window_index
        best_d = float("inf")
        for i in range(best_i, i1 + 1):
            px, py = self.route_xy[i]
            d = math.hypot(px - cx, py - cy)
            if d < best_d:
                best_d = d
                best_i = i

        self.window_index = best_i

        start = self.window_index
        end = min(start + self.N, len(self.route_xy))
        window = list(self.route_xy[start:end])

        if 0 < len(window) < self.N:
            window.extend([window[-1]] * (self.N - len(window)))

        return window

    def _map_points_to_base_link(self, pts_map, base_pose_map):
        bx, by, yaw = base_pose_map
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        pts_base = []
        for px, py in pts_map:
            dx = px - bx
            dy = py - by
            x_b =  cy * dx + sy * dy
            y_b = -sy * dx + cy * dy
            pts_base.append((x_b, y_b))
        return pts_base

    def _publish_drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.cmd_pub.publish(msg)

    def control_tick(self):
        if self._latest_odom is None or not self.route_xy:
            self._publish_drive(0.0, 0.0)
            return

        if self.last_odom_time is None or (self.get_clock().now() - self.last_odom_time) > Duration(seconds=0.25):
            self._publish_drive(0.0, 0.0)
            return

        if not self.tf_buffer.can_transform(
                self.global_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.2)):
            self.get_logger().warn("Waiting for TF %s->%s..." % (self.global_frame, self.base_frame))
            self._publish_drive(0.0, 0.0)
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn("TF lookup failed %s->%s: %s" % (self.global_frame, self.base_frame, str(e)))
            self._publish_drive(0.0, 0.0)
            return

        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        q = tf.transform.rotation
        yaw, _, _ = quaternion_to_euler(q.x, q.y, q.z, q.w)

        print("bx", bx)
        print("by", by)

        window_map = self._select_window((bx, by))
        print(window_map, "window")

        if len(window_map) < 2:
            self._publish_drive(0.0, 0.0)
            return

        window_base = self._map_points_to_base_link(window_map, (bx, by, yaw))
        print(window_base, "window_base")
        print(yaw, "yaw")

        steer = self.controller_PP.compute(window_base)

        max_steer = 0.35
        steer = max(min(steer, max_steer), -max_steer)

        speed = 0.35
        self._publish_drive(speed, steer)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
