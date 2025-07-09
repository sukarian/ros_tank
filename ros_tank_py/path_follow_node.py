import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Twist
from std_msgs.msg import Float32MultiArray, Float32, Bool
import math
import numpy as np
from tf_transformations import euler_from_quaternion


#from ros_tank_py.srv import Waypoint
import numpy as np

class PathFollowNode(Node):

	def __init__(self):
		super().__init__('path_follow_node')
		self.path_sub = self.create_subscription(Float32MultiArray, '/path', self.path_callback, 10)
		self.pose_sub = self.create_subscription(Pose, '/estimated_pose', self.pose_callback, 10)
		self.yaw_tgt_pub = self.create_publisher(Float32, '/yaw_tgt', 10)
		self.path_done_pub = self.create_publisher(Bool, '/path_done', 10)
		self.position = []
		self.orientation = []
		self.pose_rxd = False
		self.path = Float32MultiArray()
		self.lookahead_dist = 0.15
		self.last_point = False
		self.path_rxd = False
		self.path_done = Bool()
		self.target_yaw = Float32()
		self.timer = self.create_timer(0.1, self.timer_callback)

	def pose_callback(self, msg):
		self.position = [msg.position.x,
				msg.position.y,
				msg.position.z]

		orient = msg.orientation
		_, _, self.yaw = euler_from_quaternion([msg.orientation.x,
				msg.orientation.y,
				msg.orientation.z,
				msg.orientation.w])
		self.pose_rxd = True


	def find_lookahead_point(self):
		# Find the closest point on the path
		#distances = np.linalg.norm(self.path - self.position, axis=1)
		#closest_idx = np.argmin(distances)
		last_distance = 10000
		for i in range(len(self.path)):
			distance = np.linalg.norm(self.path[i] - self.position)
			if distance > last_distance:
				break
			closest_idx = i
			#self.get_logger().info(f"{closest_idx}")
			last_distance = distance
		if closest_idx > 0:
			self.path = self.path[closest_idx-1:]
		#self.get_logger().info(f"Path Length: {len(self.path)}")

		# Start searching from the closest point onward
		for i in range(len(self.path)):
			dist = np.linalg.norm(self.path[i] - self.position)
			if dist >= self.lookahead_dist:
				# Found the lookahead point
				return self.path[i], False

		# If no point is found beyond lookahead distance, return the last point
		return self.path[-1], True

	def path_callback(self, msg):
		"""Callback for receiving the path as Float32MultiArray"""
		# The path is expected to be a sequence of [x1,y1,z1, x2,y2,z2, ...]
		if not self.path_rxd:
			data = np.array(msg.data)
			self.path = data.reshape(-1, 3)  # Reshape into Nx3 array
			self.path_rxd = True
			#self.get_logger().info(f"Received new path with {len(self.path)} points")

	def yaw_limit(self):
		if self.yaw_tgt.data  < -math.pi:
			self.yaw_tgt.data += 2 * math.pi
		elif self.yaw_tgt.data > math.pi:
			self.yaw_tgt.data -= 2 * math.pi

	def calculate_yaw_tgt(self, lookahead_point):
		if lookahead_point is None:
			return None

		dx = lookahead_point[0] - self.position[0]
		dy = lookahead_point[1] - self.position[1]

		target_yaw = math.atan2(dy, dx)

		return target_yaw

	def path_follow(self):
		if not self.path_rxd:
			self.get_logger().info(f"No Path!")
			return

		if not self.pose_rxd:
			self.get_logger().info(f"No Pose!")
			return

		if self.path_done.data:
			self.path_done_pub.publish(self.path_done)
			return

		lookahead_point, last_point = self.find_lookahead_point()
		self.get_logger().info(f"Path done: {last_point}")
		self.target_yaw.data = self.calculate_yaw_tgt(lookahead_point)

		if lookahead_point is None:
			return
		self.get_logger().info(f"Lookahead Point: {lookahead_point[0]}, {lookahead_point[1]}")
		#self.get_logger().info(f"Yaw Target: {self.target_yaw}")

		self.path_done.data = last_point
		self.yaw_tgt_pub.publish(self.target_yaw)
		self.path_done_pub.publish(self.path_done)

	def timer_callback(self):
		#self.get_logger().info('Following Path')
		self.path_follow()

def main(args=None):
	rclpy.init(args=args)
	path_follow_node = PathFollowNode()
	rclpy.spin(path_follow_node)
	path_follow_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
