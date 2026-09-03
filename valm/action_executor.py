import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from valm.robot import Robot

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Quaternion

import math

class ActionExecutor(Node):

    def __init__(self):
        super().__init__("action_executor")
        
        self.scene = {"objects": []}
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.robot = Robot(self, callback_group = self.callback_group)

        self.create_subscription(String, "/scene_state", self.scene_callback, 10, callback_group=self.callback_group)

        self.create_subscription(String, "/action_plan", self.plan_callback, 10, callback_group=self.callback_group)
        
        self.get_logger().info("Action executor started")

    def scene_callback(self, msg):

        try:
            self.scene = json.loads(msg.data)

        except json.JSONDecodeError as error:
            self.get_logger().error(f"Invalid scene JSON: {error}")

    async def plan_callback(self, msg):

        try:
            plan = json.loads(msg.data)

        except json.JSONDecodeError as error:
            self.get_logger().error(f"Invalid action plan JSON: {error}")
            return

        actions = plan.get("actions", [])

        self.get_logger().info(f"Received plan with "f"{len(actions)} actions")

        for action in actions:

            if not await self.execute_action(action):
                self.get_logger().error("Action execution failed")
                break

    async def execute_action(self, action):

        action_name = action.get("action")

        if action_name == "move_relative":

            return await self.execute_move_relative(action)

        elif action_name == "pick":

            return await self.execute_pick(action)

        elif action_name == "place":

            return await self.execute_place(action)

        elif action_name == "stop":

            self.get_logger().warning("Stop action received")

            return False

        else:

            self.get_logger().error(f"Unknown action: "f"{action_name}")

            return False

    async def execute_move_relative(self, action):

        x = float(action.get("x", 0.0))
        y = float(action.get("y", 0.0))
        z = float(action.get("z", 0.0))

        self.get_logger().info(f"move_relative: "f"x={x:.3f}, "f"y={y:.3f}, "f"z={z:.3f}")

        try:

            success = await self.robot.move_relative(x,y,z)

            if not success:
                self.get_logger().error("Robot failed to execute relative movement")
                return False

            self.get_logger().info("Relative movement completed")
            return True

        except Exception as error:

            self.get_logger().error(f"move_relative failed: "f"{type(error).__name__}: {error}")

            return False

    async def execute_pick(self, action):

        object_id = action.get("object_id")

        obj = self.get_object(object_id)

        if obj is None:

            self.get_logger().error(f"Object not found: "f"{object_id}")

            return False

        position = obj.get("position")
        
        if (position is None or len(position) != 3):
            self.get_logger().error(f"Invalid position for {object_id}")
            return False

        object_x = float(position[0])
        object_y = float(position[1])
        object_z = float(position[2])
        
        grip_position = obj.get("grip_position")

        if grip_position is not None and len(grip_position) == 3:
            grasp_x = float(grip_position[0])
            grasp_y = float(grip_position[1])

            self.get_logger().info(f"Using grip midpoint: "f"x={grasp_x:.4f}, y={grasp_y:.4f}")

        else:
            grasp_x = object_x
            grasp_y = object_y

            self.get_logger().warn(f"No grip_position for {object_id}; "f"using object centre")
        
        grip_width = obj.get("grip_width")
        grip_angle_base = obj.get("grip_angle_base")

        if grip_width is None:
            self.get_logger().error(f"No grip_width available for {object_id}")
            return False

        if grip_angle_base is None:
            self.get_logger().error(f"No grip_angle_base available for {object_id}")
            return False

        grip_width = float(grip_width)
        grip_angle_base = float(grip_angle_base)
        

        self.get_logger().info(f"pick: "f"{object_id}, "f"label={obj.get('label')}, "f"position={position}, "f"grip_width={grip_width:.3f} m, "f"grip_angle_base={grip_angle_base:.1f} deg")
        
        # Pick parameters 
        approach_height = 0.30
        grasp_offset = 0.02
        lift_height = 0.20

        # Open gripper
        self.get_logger().info("Opening gripper")

        if not await self.robot.open_gripper():
            self.get_logger().error("Failed to open gripper")
            return False
            
        # Move above object
        approach_z = (object_z + approach_height)
        
        current_pose = self.robot.get_current_pose()

        if current_pose is None:
            self.get_logger().error("Could not get current robot pose")
            return False

        q = current_pose.orientation

        
        # Current yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Vision grip angle + required gripper offset
        desired_yaw = math.radians(grip_angle_base + 90.0)

        # Normalize angular difference to [-pi, pi]
        def angle_difference(target, current):
            return math.atan2(math.sin(target - current), math.cos(target - current))

        # A parallel-jaw gripper has 180-degree symmetry.
        # Both of these represent the same grasp.
        candidate_1 = desired_yaw
        candidate_2 = desired_yaw + math.pi

        difference_1 = angle_difference(candidate_1, current_yaw)

        difference_2 = angle_difference(candidate_2, current_yaw)

        # Choose whichever requires the least wrist rotation
        if abs(difference_1) <= abs(difference_2):
            yaw = current_yaw + difference_1
        else:
            yaw = current_yaw + difference_2

        self.get_logger().info(f"Current yaw={math.degrees(current_yaw):.1f} deg, "f"grip angle={grip_angle_base:.1f} deg, "f"selected yaw={math.degrees(yaw):.1f} deg")

        # Downward-facing gripper
        half_yaw = yaw / 2.0

        orientation = Quaternion()
        orientation.x = math.cos(half_yaw)
        orientation.y = math.sin(half_yaw)
        orientation.z = 0.0
        orientation.w = 0.0
        
        self.get_logger().info(f"Moving above object: "f"x={grasp_x:.3f}, "f"y={grasp_y:.3f}, "f"z={approach_z:.3f}")

        if not await self.robot.move_to_pose(grasp_x,grasp_y,approach_z,orientation):
            self.get_logger().error("Failed to move above object")
            return False
            
        # TEMPORARY
        #self.get_logger().info("Approach test completed")
        #return True

        # Move down to grasp
        eef_to_tcp_offset = 0.172

        # How far below the detected object point
        # we would normally try to grasp
        grasp_depth = 0.020

        # Keep the bottom of the gripper this far
        # above the locally measured surface
        surface_safety_margin = 0.000


        # Normal grasp position
        desired_grasp_z = (object_z + eef_to_tcp_offset - grasp_depth)

        # Get the local surface height measured
        # beside this specific object
        surface_z = obj.get("surface_z")


        if surface_z is not None:

            surface_z = float(surface_z)

            # Lowest safe EEF position.
            # This prevents the gripper from descending
            # too close to / into the table or surface.
            minimum_safe_grasp_z = (surface_z + eef_to_tcp_offset + surface_safety_margin)

            # Use whichever value keeps the robot higher
            grasp_z = max(desired_grasp_z, minimum_safe_grasp_z)

            self.get_logger().info(f"Grasp height calculation: "f"object_z={object_z:.4f} m, "f"surface_z={surface_z:.4f} m, "f"desired_z={desired_grasp_z:.4f} m, "f"minimum_safe_z={minimum_safe_grasp_z:.4f} m, "f"final_z={grasp_z:.4f} m")

        else:

            # Fall back to normal grasp calculation
            # if vision did not provide surface_z
            grasp_z = desired_grasp_z

            self.get_logger().warn(f"No surface_z available for {object_id}; "f"using normal grasp depth")

        self.get_logger().info(f"Moving to grasp position: "f"x={object_x:.3f}, "f"y={object_y:.3f}, "f"z={grasp_z:.3f}")

        if not await self.robot.move_to_pose(grasp_x, grasp_y, grasp_z, orientation):
            self.get_logger().error("Failed to move to grasp position")
            return False
            
        # TEMPORARY
        #self.get_logger().info("Approach test completed")
        #return True
            
        # Close gripper
        gripper_position = self.robot.width_to_gripper_position(grip_width)

        self.get_logger().info(f"Closing gripper for object width "f"{grip_width * 1000:.1f} mm -> "f"drive_joint={gripper_position:.3f}")

        if not await self.robot.close_gripper(gripper_position):
            self.get_logger().error("Failed to close gripper")
            return False
            
        max_gripper_width = 0.086

        if grip_width > max_gripper_width:
            self.get_logger().error(f"Cannot grasp {object_id}: "f"object width={grip_width * 1000:.1f} mm, "f"gripper maximum={max_gripper_width * 1000:.1f} mm")
            return False
            
        # Lift object
        lift_z = (grasp_z + lift_height)

        self.get_logger().info(f"Lifting object to "f"z={lift_z:.3f}")

        if not await self.robot.move_to_pose(grasp_x,grasp_y,lift_z,orientation):
            self.get_logger().error("Failed to lift object")
            return False

        self.get_logger().info(f"Pick completed: {object_id}")

        return True

    def execute_place(self, action):

        object_id = action.get("object_id")

        target = action.get("target")

        self.get_logger().info(f"place: "f"{object_id} -> {target}")

        # Robot place will be added later

        return True

    def get_object(self, object_id):

        for obj in self.scene.get("objects", []):

            if obj.get("id") == object_id:
                return obj

        return None


def main(args=None):

    rclpy.init(args=args)

    node = ActionExecutor()

    executer = MultiThreadedExecutor(num_threads=4)
    executer.add_node(node)
    
    try:
        executer.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executer.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
