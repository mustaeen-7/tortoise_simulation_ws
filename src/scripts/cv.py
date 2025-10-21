#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

class MoveAndCapture(Node):
    def __init__(self):
        super().__init__('move_and_capture')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.image_saved = False

    def image_callback(self, msg):
        if not self.image_saved:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.expanduser(f'~/captured_image_{int(time.time())}.png')
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'✅ Image saved: {filename}')
            self.image_saved = True

    def move(self, linear_vel, duration):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = 0.0
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        # stop after moving
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = MoveAndCapture()

    node.get_logger().info('🚀 Moving forward...')
    node.move(0.2, 3.0)  # move forward for 3 seconds

    node.get_logger().info('📸 Capturing image...')
    # Spin briefly to ensure we receive at least one image
    timeout = node.get_clock().now().seconds_nanoseconds()[0] + 5
    while not node.image_saved and node.get_clock().now().seconds_nanoseconds()[0] < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('⬅️ Moving back...')
    node.move(-0.2, 3.0)  # move backward for 3 seconds

    node.get_logger().info('✅ Task complete. Robot returned to spawn.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#sudo apt install ros-humble-cv-bridge python3-opencv
