from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes,)

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class Robot:
    def __init__(self, node, base_frame="link_base", end_effector_frame="link_eef", callback_group=None):
    
        self.node = node
        
        # Robot frames
        self.base_frame = base_frame
        self.end_effector_frame = end_effector_frame
        
        # Prev message
        self.joint_state = None
        
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_efforts = {}
        
        # Subscibes to joint states from ros
        self.joint_state_sub = self.node.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10, callback_group=callback_group)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self.planning_group = "xarm5"
        self.move_group_client = ActionClient(self.node, MoveGroup, "/move_action")
        
        self.gripper_client = ActionClient(self.node, FollowJointTrajectory, "/xarm_gripper_traj_controller/follow_joint_trajectory", callback_group=callback_group)
        
        self.ik_client = self.node.create_client(GetPositionIK, "/compute_ik")
        
        self.node.get_logger().info("Robot interface initialized.")
        
        
        
    async def compute_ik(self, x, y, z, orientation):

        if not self.ik_client.service_is_ready(): 
            self.node.get_logger().info("Waiting for /compute_ik service...")

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("/compute_ik service not available")
            return None

        request = GetPositionIK.Request()

        request.ik_request.group_name = self.planning_group
        request.ik_request.ik_link_name = self.end_effector_frame

        request.ik_request.pose_stamped.header.frame_id = self.base_frame

        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z

        request.ik_request.pose_stamped.pose.orientation = orientation

        request.ik_request.avoid_collisions = False

        request.ik_request.timeout.sec = 2
        request.ik_request.timeout.nanosec = 0

        self.node.get_logger().info("Requesting IK solution...")

        response = await self.ik_client.call_async(request)

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(f"IK failed with error code: "f"{response.error_code.val}")
            return None

        joint_state = response.solution.joint_state

        joint_positions = dict(zip(joint_state.name, joint_state.position))

        self.node.get_logger().info("IK solution found: "f"{joint_positions}")
        
        return joint_positions
    
        
    def joint_state_callback(self, msg: JointState):
        
        self.joint_state = msg
        
        self.joint_positions = dict(zip(msg.name, msg.position))
        self.joint_velocities = dict(zip(msg.name, msg.velocity))
        self.joint_efforts = dict(zip(msg.name, msg.effort))
        
    def get_current_pose(self):
    
        try:
            transform = (self.tf_buffer.lookup_transform(self.base_frame, self.end_effector_frame, Time()))
            pose = Pose()
            
            # Positions
            pose.position.x = (transform.transform.translation.x)
            pose.position.y = (transform.transform.translation.y)
            pose.position.z = (transform.transform.translation.z)
            
            # Orientations
            pose.orientation.x = (transform.transform.rotation.x)
            pose.orientation.y = (transform.transform.rotation.y)
            pose.orientation.z = (transform.transform.rotation.z)
            pose.orientation.w = (transform.transform.rotation.w)
            
            return pose
            
        except Exception as error:
        
            self.node.get_logger().error(f"Failed to get end-effector pose: "f"{type(error).__name__}: {error}")
            return None
        
    async def move_relative(self, dx, dy, dz):

        current_pose = self.get_current_pose()
            
        if current_pose is None:
            self.node.get_logger().error("Cannot move relative: ""current pose unavailable")
            return False

        target_x = current_pose.position.x + dx
        target_y = current_pose.position.y + dy
        target_z = current_pose.position.z + dz
            
        self.node.get_logger().info(f"Current EE position: "f"x={current_pose.position.x:.3f}, "f"y={current_pose.position.y:.3f}, "f"z={current_pose.position.z:.3f}")
                    
        self.node.get_logger().info(f"Relative movement: "f"dx={dx:.3f}, "f"dy={dy:.3f}, "f"dz={dz:.3f}")
                    
        self.node.get_logger().info(f"Target EE position: "f"x={target_x:.3f}, "f"y={target_y:.3f}, "f"z={target_z:.3f}")

        return await self.move_to_pose(target_x,target_y,target_z,current_pose.orientation)
            
    async def move_to_pose(self, x, y, z, orientation):

        self.node.get_logger().info(f"Requested move_to_pose: "f"x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Solve inverse kinematics
    
        joint_positions = await self.compute_ik(x,y,z,orientation)

        if joint_positions is None:
            return False

        # Build joint-space goal
   
        goal = MoveGroup.Goal()

        goal.request.group_name = self.planning_group
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnect"

        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0

        goal.request.max_velocity_scaling_factor = 0.20
        goal.request.max_acceleration_scaling_factor = 0.20

        constraints = Constraints()

        # Only constrain the actual xArm5 joints.
        arm_joints = ["joint1","joint2","joint3","joint4","joint5",]

        for joint_name in arm_joints:

            if joint_name not in joint_positions:
                self.node.get_logger().error(f"IK solution missing {joint_name}")
                return False

            joint_constraint = JointConstraint()

            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_positions[joint_name]

            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001

            joint_constraint.weight = 1.0

            constraints.joint_constraints.append(joint_constraint)

        goal.request.goal_constraints.append(constraints)

  
        # Plan and execute

        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True

        if not self.move_group_client.server_is_ready():
            self.node.get_logger().info("Waiting for MoveIt /move_action server...")

        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveIt action server not available")
            return False

        self.node.get_logger().info("Sending joint-space goal to MoveIt...")

        goal_handle = await self.move_group_client.send_goal_async(goal)

        if not goal_handle.accepted:
            self.node.get_logger().error("MoveIt rejected the goal")
            return False

        self.node.get_logger().info("MoveIt goal accepted")

        result_response = await goal_handle.get_result_async()
        result = result_response.result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:

            self.node.get_logger().error(f"MoveIt failed with error code: "f"{result.error_code.val}")
            return False

        self.node.get_logger().info(f"MoveIt motion completed successfully. "f"Planning time: {result.planning_time:.3f} s")
        return True


    async def set_gripper(self, position):

        if not self.gripper_client.server_is_ready():

            self.node.get_logger().info("Waiting for gripper controller...")

            if not self.gripper_client.wait_for_server(timeout_sec=5.0):
                self.node.get_logger().error("Gripper controller unavailable")

                return False

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = ["drive_joint"]

        point = JointTrajectoryPoint()

        point.positions = [float(position)]

        point.time_from_start.sec = 5
        point.time_from_start.nanosec = 0

        goal.trajectory.points.append(point)

        self.node.get_logger().info(f"Sending gripper target: {position:.3f}")

        goal_handle = await self.gripper_client.send_goal_async(goal)

        if not goal_handle.accepted:
            self.node.get_logger().error("Gripper goal rejected")
            return False

        result_response = await goal_handle.get_result_async()

        self.node.get_logger().info(f"Gripper movement completed: {position:.3f}")

        return True
            
    def width_to_gripper_position(self, width_m, grip_compression=0.004):

        max_opening = 0.086
        max_joint_position = 0.85

        # Close slightly further than the measured object width
        target_width = width_m - grip_compression

        # Prevent invalid widths
        target_width = max(0.0, min(target_width, max_opening))

        # Convert opening width to drive_joint position
        position = max_joint_position * (1.0 - (target_width / max_opening))

        return position
        
    async def open_gripper(self):
        return await self.set_gripper(0.0)

    async def close_gripper(self, position=0.85):
        return await self.set_gripper(position)
        #return await self.set_gripper(0.3)

    def get_joint_positions(self):
        return self.joint_positions.copy()
        
    def get_joint_velocities(self):
        return self.joint_velocities.copy()
    
    def get_joint_efforts(self):
        return self.joint_efforts.copy()
        
    def get_joint_names(self):
        if self.joint_state is None:
            return []
        return list(self.joint_state.name)
        
    def get_joint_state(self):
        return self.joint_state
        
    def has_state(self):
        return self.joint_state is not None
        
        

        
