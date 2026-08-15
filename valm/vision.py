import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision')

        self.bridge = CvBridge()

        # Load trained YOLO segmentation model
        self.model = YOLO(
            '/home/sam/dev_ws/src/valm/training_runs/'
            'food_seg_v5_sim/weights/best.pt'
        )

        # Subscribe to Gazebo camera
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publish annotated YOLO image
        self.annotated_pub = self.create_publisher(
            Image,
            '/vision/annotated_image',
            10
        )

        self.get_logger().info('Vision node started')

    def image_callback(self, msg):

        try:
            # Convert ROS image -> OpenCV image
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # Run YOLO instance segmentation
            results = self.model.predict(
                source=frame,
                conf=0.25,
                device=0,
                verbose=False
            )

            result = results[0]

            # Draw segmentation masks, bounding boxes,
            # class names and confidence values
            annotated_frame = result.plot()

            # Build ROS Image message manually.
            # This avoids cv_bridge.cv2_to_imgmsg()
            # compatibility issues.
            annotated_msg = Image()

            annotated_msg.header = msg.header
            annotated_msg.height = annotated_frame.shape[0]
            annotated_msg.width = annotated_frame.shape[1]
            annotated_msg.encoding = 'bgr8'
            annotated_msg.is_bigendian = False
            annotated_msg.step = annotated_frame.shape[1] * 3
            annotated_msg.data = annotated_frame.tobytes()

            # Publish annotated image
            self.annotated_pub.publish(annotated_msg)

            # Log detected objects
            if result.boxes is not None:

                for box in result.boxes:

                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])

                    class_name = self.model.names[class_id]

                    self.get_logger().info(
                        f'{class_name}: {confidence:.2f}'
                    )

        except Exception as e:

            self.get_logger().error(
                f'Vision callback failed: '
                f'{type(e).__name__}: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = VisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
