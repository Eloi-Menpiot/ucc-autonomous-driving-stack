#!/usr/bin/env python3
"""
return_to_origin.py — Biased Follow the Gap that drives back to a goal point.
Uses odometry only, no map or localization needed.

USAGE:
    ros2 run autonomous_driving_pkg return_to_origin --ros-args \
        -p goal_x:=0.0 -p goal_y:=0.0 -p max_speed:=0.5
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


def yaw_from_quat(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class ReturnToOrigin(Node):
    def __init__(self):
        super().__init__("return_to_origin")

        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tolerance", 1.0)
        self.declare_parameter("slowdown_radius", 3.0)
        self.declare_parameter("max_speed", 0.8)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("crawl_speed", 0.3)
        self.declare_parameter("max_steer", 0.35)
        self.declare_parameter("bubble_radius", 0.5)
        self.declare_parameter("max_range_clip", 3.0)
        self.declare_parameter("min_gap_size", 10)
        self.declare_parameter("disparity_threshold", 0.5)
        self.declare_parameter("window_size", 5)
        self.declare_parameter("fov_deg", 180.0)
        self.declare_parameter("goal_weight", 0.6)

        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.slowdown_radius = self.get_parameter("slowdown_radius").value
        self.max_speed = self.get_parameter("max_speed").value
        self.min_speed = self.get_parameter("min_speed").value
        self.crawl_speed = self.get_parameter("crawl_speed").value
        self.max_steer = self.get_parameter("max_steer").value
        self.bubble_radius = self.get_parameter("bubble_radius").value
        self.max_range_clip = self.get_parameter("max_range_clip").value
        self.min_gap_size = self.get_parameter("min_gap_size").value
        self.disparity_threshold = self.get_parameter("disparity_threshold").value
        self.window_size = self.get_parameter("window_size").value
        self.fov_deg = self.get_parameter("fov_deg").value
        self.goal_weight = self.get_parameter("goal_weight").value

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.has_odom = False
        self.reached_goal = False
        self.log_counter = 0

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, "/ackermann_cmd", 10)

        self.get_logger().info(
            "Return to Origin | goal=(%.2f, %.2f) tolerance=%.2fm "
            "bubble=%.2fm speed=[%.1f-%.1f]"
            % (self.goal_x, self.goal_y, self.goal_tolerance,
               self.bubble_radius, self.min_speed, self.max_speed))

    def odom_callback(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.car_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.has_odom = True

    def scan_callback(self, msg):
        if not self.has_odom:
            self.publish_drive(0.0, 0.0)
            return

        dist_to_goal = math.hypot(
            self.goal_x - self.car_x, self.goal_y - self.car_y)

        if dist_to_goal < self.goal_tolerance:
            if not self.reached_goal:
                self.reached_goal = True
                self.get_logger().info(
                    "Within %.1fm of origin (dist=%.2fm). Stopping."
                    % (self.goal_tolerance, dist_to_goal))
            self.publish_drive(0.0, 0.0)
            return

        self.reached_goal = False

        goal_bearing_world = math.atan2(
            self.goal_y - self.car_y, self.goal_x - self.car_x)
        goal_bearing_local = normalize_angle(goal_bearing_world - self.car_yaw)

        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)

        if n == 0:
            self.publish_drive(0.0, 0.0)
            return

        fov_rad = math.radians(self.fov_deg)
        half_fov = fov_rad / 2.0
        angles = np.linspace(msg.angle_min, msg.angle_max, n)
        fov_mask = np.abs(angles) <= half_fov

        proc = np.copy(ranges)
        proc[~np.isfinite(proc)] = 0.0
        proc = np.clip(proc, 0.0, self.max_range_clip)
        proc[~fov_mask] = 0.0

        if self.window_size > 1:
            kernel = np.ones(self.window_size) / self.window_size
            proc = np.convolve(proc, kernel, mode='same')

        # Safety bubble
        fov_ranges = np.copy(proc)
        fov_ranges[~fov_mask] = self.max_range_clip + 1
        fov_ranges[fov_ranges <= 0.01] = self.max_range_clip + 1
        closest_idx = np.argmin(fov_ranges)

        closest_range = proc[closest_idx]
        if closest_range > 0.01:
            bubble_angle = math.atan2(self.bubble_radius, closest_range)
            bubble_beams = int(bubble_angle / msg.angle_increment)
            lo = max(0, closest_idx - bubble_beams)
            hi = min(n - 1, closest_idx + bubble_beams)
            proc[lo:hi + 1] = 0.0

        # Emergency stop
        if closest_range < 0.2 and closest_range > 0.01:
            self.get_logger().warn(
                "Emergency! Obstacle at %.2fm. Stopping." % closest_range)
            self.publish_drive(0.0, 0.0)
            return

        # Disparity extender
        for i in range(1, n):
            diff = abs(proc[i] - proc[i - 1])
            if diff > self.disparity_threshold:
                shorter_range = min(proc[i], proc[i - 1])
                if shorter_range > 0.01:
                    extend_angle = math.atan2(self.bubble_radius, shorter_range)
                    extend_beams = int(extend_angle / msg.angle_increment)
                    if proc[i] < proc[i - 1]:
                        end = min(n, i + extend_beams)
                        proc[i:end] = np.minimum(proc[i:end], shorter_range)
                    else:
                        start = max(0, i - extend_beams)
                        proc[start:i] = np.minimum(proc[start:i], shorter_range)

        # Find largest gap
        threshold = 0.1
        open_mask = proc > threshold

        best_gap_start = 0
        best_gap_end = 0
        best_gap_len = 0
        gap_start = None

        for i in range(n):
            if open_mask[i]:
                if gap_start is None:
                    gap_start = i
            else:
                if gap_start is not None:
                    gap_len = i - gap_start
                    if gap_len > best_gap_len:
                        best_gap_len = gap_len
                        best_gap_start = gap_start
                        best_gap_end = i - 1
                    gap_start = None

        if gap_start is not None:
            gap_len = n - gap_start
            if gap_len > best_gap_len:
                best_gap_len = gap_len
                best_gap_start = gap_start
                best_gap_end = n - 1

        if best_gap_len < self.min_gap_size:
            self.get_logger().warn("No valid gap! Stopping.")
            self.publish_drive(0.0, 0.0)
            return

        # Goal-biased gap selection
        # Clamp bearing to FOV so if goal is behind, car turns maximally
        clamped_bearing = float(np.clip(goal_bearing_local, -half_fov, half_fov))

        gap_indices = np.arange(best_gap_start, best_gap_end + 1)
        gap_angles = angles[gap_indices]
        gap_ranges = proc[gap_indices]

        angle_diffs = np.abs(gap_angles - clamped_bearing)
        angle_diffs = np.minimum(angle_diffs, 2 * np.pi - angle_diffs)
        max_angle_diff = angle_diffs.max()
        if max_angle_diff > 0:
            angle_score = 1.0 - (angle_diffs / max_angle_diff)
        else:
            angle_score = np.ones_like(angle_diffs)

        max_range = gap_ranges.max()
        if max_range > 0:
            range_score = gap_ranges / max_range
        else:
            range_score = np.ones_like(gap_ranges)

        w = self.goal_weight
        combined_score = w * angle_score + (1.0 - w) * range_score

        best_local = np.argmax(combined_score)
        best_idx = best_gap_start + best_local

        best_angle = angles[best_idx]
        steer = float(np.clip(best_angle, -self.max_steer, self.max_steer))

        # Speed control
        steer_ratio = abs(steer) / self.max_steer

        if dist_to_goal < self.slowdown_radius:
            speed_limit = self.crawl_speed + (self.max_speed - self.crawl_speed) * (
                dist_to_goal / self.slowdown_radius)
        else:
            speed_limit = self.max_speed

        speed = speed_limit - steer_ratio * (speed_limit - self.min_speed)
        speed = max(speed, self.crawl_speed)

        self.publish_drive(speed, steer)

        self.log_counter += 1
        if self.log_counter % 25 == 0:
            self.get_logger().info(
                "dist=%.2fm bearing=%.0fdeg steer=%.0fdeg speed=%.2f"
                % (dist_to_goal, math.degrees(goal_bearing_local),
                   math.degrees(steer), speed))

    def publish_drive(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReturnToOrigin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
