#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cat_follower_node.py — Follows the largest detected cat using YOLO detections.

Subscribes:
  /yolo/detections  (vision_msgs/Detection2DArray)

Publishes:
  /cmd_vel          (geometry_msgs/Twist)

Control law (proportional):
  angular.z  = -Kp_yaw  * (cx - img_cx) / img_cx     # center horizontally
  linear.x   =  Kp_dist * (target_h - bbox_h) / img_h # approach/retreat

Safety: stops if no cat detected for `timeout` seconds.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import time


class CatFollower(Node):

    def __init__(self):
        super().__init__('cat_follower')

        # --- Parameters ---
        self.declare_parameter('image_width',    640)
        self.declare_parameter('image_height',   480)
        self.declare_parameter('kp_yaw',         0.8)   # proportional gain for yaw
        self.declare_parameter('kp_dist',        0.4)   # proportional gain for distance
        self.declare_parameter('target_bbox_h',  180.0) # target bbox height in pixels (~0.8-1 m)
        self.declare_parameter('max_linear_vel', 0.15)  # m/s
        self.declare_parameter('max_angular_vel', 0.6)  # rad/s
        self.declare_parameter('min_bbox_h',     30.0)  # ignore very small detections (noise)
        self.declare_parameter('stop_timeout',   1.5)   # seconds without detection before stopping

        self.img_w        = self.get_parameter('image_width').value
        self.img_h        = self.get_parameter('image_height').value
        self.kp_yaw       = self.get_parameter('kp_yaw').value
        self.kp_dist      = self.get_parameter('kp_dist').value
        self.target_h     = self.get_parameter('target_bbox_h').value
        self.max_lin      = self.get_parameter('max_linear_vel').value
        self.max_ang      = self.get_parameter('max_angular_vel').value
        self.min_h        = self.get_parameter('min_bbox_h').value
        self.stop_timeout = self.get_parameter('stop_timeout').value

        self.last_detection_time = None

        # --- Publishers / Subscribers ---
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_det = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.detections_callback, 10)

        # Safety watchdog: check for stale detections every 0.2 s
        self.create_timer(0.2, self.watchdog_callback)

        self.get_logger().info(
            '\033[1;32mCat Follower started. Waiting for detections on /yolo/detections...\033[0m')

    # ------------------------------------------------------------------
    def detections_callback(self, msg: Detection2DArray):
        best = None  # best detection (largest cat bbox)

        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id == 'cat':
                    if det.bbox.size_y >= self.min_h:
                        if best is None or det.bbox.size_y > best.bbox.size_y:
                            best = det

        if best is None:
            return  # no cat in this frame, watchdog handles the stop

        self.last_detection_time = time.time()

        cx   = best.bbox.center.position.x
        bbox_h = best.bbox.size_y

        img_cx = self.img_w / 2.0

        # Yaw: positive error → cat is to the right → turn right (negative angular.z in ROS)
        err_yaw  = (cx - img_cx) / img_cx          # -1 … +1
        # Distance: positive error → cat too small (far) → move forward
        err_dist = (self.target_h - bbox_h) / self.img_h  # normalised

        cmd = Twist()
        cmd.angular.z = float(
            max(-self.max_ang, min(self.max_ang, -self.kp_yaw * err_yaw)))
        cmd.linear.x = float(
            max(-self.max_lin, min(self.max_lin,  self.kp_dist * err_dist)))

        self.pub_cmd.publish(cmd)
        self.get_logger().debug(
            f'cat cx={cx:.0f} h={bbox_h:.0f} → vx={cmd.linear.x:.3f} az={cmd.angular.z:.3f}')

    # ------------------------------------------------------------------
    def watchdog_callback(self):
        if self.last_detection_time is None:
            return
        elapsed = time.time() - self.last_detection_time
        if elapsed > self.stop_timeout:
            self.stop()
            self.get_logger().info(
                f'No cat detected for {elapsed:.1f}s — stopping.')
            self.last_detection_time = None  # avoid repeated log spam

    def stop(self):
        self.pub_cmd.publish(Twist())  # all-zeros = full stop


def main(args=None):
    rclpy.init(args=args)
    node = CatFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
