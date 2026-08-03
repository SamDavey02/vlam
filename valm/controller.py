#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


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

        self.get_logger().info(
            'VALM controller started!'
        )


    def image_callback(self, msg):
    	try:
        	frame = self.bridge.imgmsg_to_cv2(
            	msg,
            	desired_encoding='rgb8'
        	)

        	#self.get_logger().info(
            	#f"Received image: {frame.shape}"
        	#)
        	
        	frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        	
        	cv2.imshow("RealSense RGB", frame)
        	cv2.waitKey(1)

    	except Exception as e:
        	self.get_logger().error(
            	f"Image conversion failed: {e}"
        	)
        
    


def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
