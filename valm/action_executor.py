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

        self.get_logger().info(f"pick: "f"{object_id}, "f"label={obj.get('label')}, "f"position={position}")

        # Pick parameters 
        approach_height = 0.30
        grasp_offset = 0.02
        lift_height = 0.10

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

        # Extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)

        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        yaw = math.atan2(siny_cosp,cosy_cosp)

        # Force gripper downward:
        # roll = pi
        # pitch = 0
        # preserve current yaw
        half_yaw = yaw / 2.0

        orientation = Quaternion()

        orientation.x = math.cos(half_yaw)
        orientation.y = math.sin(half_yaw)
        orientation.z = 0.0
        orientation.w = 0.0

        self.get_logger().info(f"Moving above object: "f"x={object_x:.3f}, "f"y={object_y:.3f}, "f"z={approach_z:.3f}")

        if not await self.robot.move_to_pose(object_x,object_y,approach_z,orientation):
            self.get_logger().error("Failed to move above object")
            return False
            
        # TEMPORARY
        #self.get_logger().info("Approach test completed")
        #return True

        # Move down to grasp
        eef_to_tcp_offset = 0.172
        grasp_depth = 0.020

        grasp_z = (object_z + eef_to_tcp_offset - grasp_depth)

        self.get_logger().info(f"Moving to grasp position: "f"x={object_x:.3f}, "f"y={object_y:.3f}, "f"z={grasp_z:.3f}")

        if not await self.robot.move_to_pose(object_x, object_y, grasp_z, orientation):
            self.get_logger().error("Failed to move to grasp position")
            return False
            
        # TEMPORARY
        #self.get_logger().info("Approach test completed")
        #return True
            
        # Close gripper
        self.get_logger().info("Closing gripper")

        if not await self.robot.close_gripper():
            self.get_logger().error("Failed to close gripper")
            return False
            
        # Lift object
        lift_z = (grasp_z + lift_height)

        self.get_logger().info(f"Lifting object to "f"z={lift_z:.3f}")

        if not await self.robot.move_to_pose(object_x,object_y,lift_z,orientation):
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
