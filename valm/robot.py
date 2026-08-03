from sensor_msgs.msg import JointState

class Robot:
	def __init__(self, node):
	
		self.node = node
		
		#prev message
		self.joint_state = None
		
		self.joint_positions = {}
		self.joint_velocities = {}
		self.joint_efforts = {}
		
		#subscibes to joint states from ros
		self.joint_state_sub = self.node.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
		
		self.node.get_logger().info("Robot interface initialized.")
		
	def joint_state_callback(self, msg: JointState):
		
		self.joint_state = msg
		
		self.joint_positions = dict(zip(msg.name, msg.position))
		self.joint_velocities = dict(zip(msg.name, msg.velocity))
		self.joint_effort = dict(zip(msg.name, msg.effort))
		
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
		

		
