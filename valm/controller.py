#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

from valm.robot import Robot


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # Camera converter
        self.bridge = CvBridge()

        # Camera subscriber
        self.camera_subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            10
        )

        # Robot interface
        self.robot = Robot(self)

        # Print robot state every 2 seconds
        self.create_timer(2.0, self.print_robot_state)

        self.get_logger().info(
            'VALM controller started!'
        )


    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='rgb8'
            )

            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

            cv2.imshow(
                "RealSense RGB",
                frame
            )

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(
                f"Image conversion failed: {e}"
            )


    def print_robot_state(self):

        if not self.robot.has_state():
            self.get_logger().info(
                "Waiting for robot state..."
            )
            return

        joints = self.robot.get_joint_positions()

        self.get_logger().info(
            str(joints)
        )


def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
