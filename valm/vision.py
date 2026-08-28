import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

import cv2
import numpy as np

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

import json
from std_msgs.msg import String

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision')
        
        # So we can make sure tracked objects stay same order
        self.tracked_objects = {}
        self.next_object_id = 0

        self.tracking_distance_threshold = 0.08

        self.bridge = CvBridge()
        
        # Camera info
        self.fx = 640.5098266601562
        self.fy = 640.5098266601562
        self.cx = 640.0
        self.cy = 360.0

        # Depth image
        self.depth_image = None
        
        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
        	self.tf_buffer,
        	self
        )

        # Load trained YOLO segmentation model
        self.model = YOLO('/home/sam/dev_ws/src/valm/training_runs/''food_seg_v5_sim/weights/best.pt')

        # Subscribe to Gazebo camera
        self.subscription = self.create_subscription(Image,'/color/image_raw',self.image_callback,qos_profile_sensor_data)
        
        # Subscribe to depth image
        self.depth_subscription = self.create_subscription(Image,'/aligned_depth_to_color/image_raw',self.depth_callback,qos_profile_sensor_data)

        # Publish annotated YOLO image
        self.annotated_pub = self.create_publisher(Image,'/vision/annotated_image',10)

        self.get_logger().info('Vision node started')
        
        # Publish scene for the language model
        self.scene_publisher = self.create_publisher(String,"/scene_state",10)
        
    def assign_object_id(self, class_name, position):

        x, y, z = position

        best_id = None
        best_distance = float("inf")

        for object_id, tracked in self.tracked_objects.items():

            # Only match objects of the same class
            if tracked["label"] != class_name:
                continue

            tx, ty, tz = tracked["position"]

            distance = ((x - tx) ** 2 + (y - ty) ** 2 + (z - tz) ** 2) ** 0.5

            if (distance < self.tracking_distance_threshold and distance < best_distance):
                best_distance = distance
                best_id = object_id

        # Existing object found
        if best_id is not None:

            self.tracked_objects[best_id] = {"label": class_name, "position": position}

            return best_id

        # New object
        new_id = f"object_{self.next_object_id}"

        self.next_object_id += 1

        self.tracked_objects[new_id] = {"label": class_name, "position": position}

        return new_id
        
    def depth_callback(self, msg):
    	
    	try:
    		# Depth topic uses 16UC1
    		self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    		
    	except Exception as e:
    		
    		self.get_logger().error(f'Depth conversion failed: 'f'{type(e).__name__}: {e}')
    		
    def calculate_shortest_grip(self, mask_pixels, depth_m):

        # Convert boolean mask to uint8 for OpenCV
        mask_uint8 = (mask_pixels.astype(np.uint8)) * 255

        # Find object contour
        contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Use largest contour
        contour = max(contours, key=cv2.contourArea)

        if len(contour) < 3:
            return None

        # Minimum area rotated rectangle
        rect = cv2.minAreaRect(contour)

        (cx, cy), (width, height), angle = rect

        if width <= 0 or height <= 0:
            return None

        box = cv2.boxPoints(rect).astype(np.float32)

        # Calculate edge lengths
        edge_01 = np.linalg.norm(box[1] - box[0])
        edge_12 = np.linalg.norm(box[2] - box[1])

        # We want a line between the two opposite LONG edges.
        # The distance between those edges is the object's shortest width.

        if edge_01 < edge_12:

            # Short dimension runs between edges (0,1) and (2,3)
            p1 = (box[1] + box[2]) / 2.0
            p2 = (box[3] + box[0]) / 2.0

        else:

            # Short dimension runs between edges (1,2) and (3,0)
            p1 = (box[0] + box[1]) / 2.0
            p2 = (box[2] + box[3]) / 2.0

        # Pixel distance
        dx_pixels = p2[0] - p1[0]
        dy_pixels = p2[1] - p1[1]

        grip_width_pixels = float(np.sqrt(dx_pixels ** 2 + dy_pixels ** 2))

        # Estimate real-world size.
        
        # Horizontal pixels scale using fx
        # Vertical pixels scale using fy.
        dx_m = (dx_pixels * depth_m) / self.fx
        dy_m = (dy_pixels * depth_m) / self.fy

        grip_width_m = float(np.sqrt(dx_m ** 2 + dy_m ** 2))

        # Angle of gripping line in image
        grip_angle = float(np.degrees(np.arctan2(dy_pixels,dx_pixels)))
        grip_angle = ((grip_angle + 90.0) % 180.0) - 90.0

        return {"point1": (int(p1[0]),int(p1[1])), "point2": (int(p2[0]),int(p2[1])), "center": (int(cx), int(cy)), "width_pixels": grip_width_pixels, "width_m": grip_width_m, "angle_deg": grip_angle}

    def image_callback(self, msg):

        try:
            # Convert ROS image -> OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO instance segmentation
            results = self.model.predict(source=frame, conf=0.25, device=0, verbose=False)

            result = results[0]

            # Draw segmentation masks, bounding boxes,
            # class names and confidence values
            annotated_frame = result.plot()

            
            # Depth and XYZ calculation
            if (self.depth_image is not None and result.boxes is not None and result.masks is not None):
            
                scene_objects = []

                for i, box in enumerate(result.boxes):

                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])

                    class_name = self.model.names[class_id]

                    # Get segmentation mask

                    mask = result.masks.data[i].cpu().numpy()

                    # Resize mask to original camera resolution
                    mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)

                    mask_pixels = mask > 0.5
                    
                    

                    # Find pixels inside mask

                    ys, xs = np.where(mask_pixels)

                    if len(xs) == 0:
                        continue

                    # Representative pixel position
                    u = int(np.median(xs))
                    v = int(np.median(ys))

                    # Check depth resolution

                    if (self.depth_image.shape[0] != frame.shape[0] or self.depth_image.shape[1] != frame.shape[1]):

                        self.get_logger().warn('RGB and depth image sizes do not match')
                        continue

                    # Get depth values inside YOLO mask

                    depth_values = self.depth_image[mask_pixels]

                    # Remove invalid zero depth
                    depth_values = depth_values[depth_values > 0]

                    if len(depth_values) == 0:

                        self.get_logger().warn(f'No valid depth for {class_name}')
                        continue

                    # Calculate representative depth

                    depth_mm = np.median(depth_values)

                    # 16UC1 depth:
                    # millimetres -> metres
                    Z = float(depth_mm) / 1000.0
                    
                                 
                    # Calculate shortest gripping width
                    
                    grip = self.calculate_shortest_grip(mask_pixels, Z)

                    if grip is not None:

                        # Draw shortest grip line
                        cv2.line(annotated_frame, grip["point1"], grip["point2"], (0, 255, 255), 3)

                        # Draw endpoints
                        cv2.circle(annotated_frame, grip["point1"], 5, (0, 0, 255), -1)

                        cv2.circle(annotated_frame, grip["point2"], 5, (0, 0, 255), -1)

                        # Label showing physical width
                        label_position = (grip["center"][0] + 10, grip["center"][1])

                        cv2.putText(annotated_frame, f'{grip["width_m"] * 1000:.1f} mm', label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                        # Log result
                        self.get_logger().info(f'{class_name}: 'f'shortest grip = 'f'{grip["width_pixels"]:.1f} px, 'f'{grip["width_m"] * 1000:.1f} mm, 'f'angle={grip["angle_deg"]:.1f} deg')
                    

                    # Pixel -> camera XYZ

                    X = ((u - self.cx)* Z / self.fx)

                    Y = ((v - self.cy)* Z / self.fy)
                    
                    # Create point stamped camera frame
                    
                    camera_point = PointStamped()

                    camera_point.header.frame_id = ('camera_color_optical_frame')

                    # Use latest available transform
                    camera_point.header.stamp = (Time().to_msg())

                    camera_point.point.x = X
                    camera_point.point.y = Y
                    camera_point.point.z = Z
                    
                    # Transform camera point -> link_base

                    try:

                        base_point = self.tf_buffer.transform(camera_point, 'link_base')

                        base_x = base_point.point.x
                        base_y = base_point.point.y
                        base_z = base_point.point.z
                        
                        
                        # Defines each object with set object ID based on location
                        position = [float(base_x), float(base_y), float(base_z)]

                        object_id = self.assign_object_id(class_name, position)

                        scene_object = {"id": object_id, "label": class_name, "confidence": confidence, "position": position}
                        
                        if grip is not None:
                            scene_object["grip_width"] = float(grip["width_m"])
                            scene_object["grip_angle"] = float(grip["angle_deg"])
                            
                        scene_objects.append(scene_object)

                        # Print
                        #self.get_logger().info(f'{class_name}: 'f'conf={confidence:.2f}, 'f'pixel=({u},{v}), 'f'camera XYZ=('f'{X:.3f}, 'f'{Y:.3f}, 'f'{Z:.3f}) m, 'f'base XYZ=('f'{base_x:.3f}, 'f'{base_y:.3f}, 'f'{base_z:.3f}) m')
                    
                    except Exception as tf_error:

                        self.get_logger().warn(f'TF transform failed for 'f'{class_name}: 'f'{tf_error}')
                        
                
                         
                # Publish full scene after processing all detections 
                scene_msg = String()

                scene_msg.data = json.dumps({"objects": scene_objects})

                self.scene_publisher.publish(scene_msg)

            # Log detected objects
            #if result.boxes is not None:

                #for box in result.boxes:

                    #class_id = int(box.cls[0])
                    #confidence = float(box.conf[0])

                    #class_name = self.model.names[class_id]

                    #self.get_logger().info(
                        #f'{class_name}: {confidence:.2f}'
                    #)
                    
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

        except Exception as e:

            self.get_logger().error(f'Vision callback failed: 'f'{type(e).__name__}: {e}')
            
        


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
