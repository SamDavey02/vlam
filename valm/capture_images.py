import os
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageCaptureNode(Node):

    def __init__(self):
        super().__init__('image_capture')

        self.bridge = CvBridge()

        self.output_dir = os.path.expanduser(
            '~/dev_ws/src/valm/gazebo_dataset/images'
        )

        os.makedirs(self.output_dir, exist_ok=True)

        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.frame_count = 0
        self.saved_count = 0

        # Save one out of every 30 frames
        self.save_every = 30

        self.get_logger().info(
            f'Saving Gazebo images to: {self.output_dir}'
        )

    def image_callback(self, msg):

        self.frame_count += 1

        # Don't save every camera frame
        if self.frame_count % self.save_every != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            filename = os.path.join(
                self.output_dir,
                f'gazebo_{self.saved_count:05d}.jpg'
            )

            cv2.imwrite(filename, frame)

            self.saved_count += 1

            self.get_logger().info(
                f'Saved {filename}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to save image: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ImageCaptureNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
