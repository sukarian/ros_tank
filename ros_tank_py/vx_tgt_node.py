import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np

class VelocityTgtNode(Node):

	def __init__(self):
		super().__init__('vx_tgt_node')
		self.pose_sub = self.create_subscription(Pose, '/estimated_pose', self.pose_callback, 10)
		self.path_sub = self.create_subscription(Float32MultiArray, '/path', self.path_callback, 10)
		self.vx_tgt_pub = self.create_publisher(Float32, '/vx_tgt', 10)
		self.timer = self.create_timer(0.1, self.timer_callback)

	def path_callback(self, msg):
		"""Callback for receiving the path as Float32MultiArray"""
		# The path is expected to be a sequence of [x1,y1,z1, x2,y2,z2, ...]
		data = np.array(msg.data)
		self.path = data.reshape(-1, 3)  # Reshape into Nx3 array
		self.path_received = True
		self.get_logger().info(f"Received new path with {len(self.path)} points")

	def calculate_target_vx(self):
		if self.path_receivedd is None:
			return None

		vx_tgt = 0.5
		slowdown_dist = 1
		end_dist = np.linalg.norm(self.path[-1] - self.current_position)

		if end_dist < slowdown_dist:
			return vx_tgt * end_dist/slowdown_dist
		return vx_tgt

	def timer_callback(self):
		vx_tgt = self.calculate_target_vx()
		self.vx_tgt_pub(vx_tgt)

def main(args=None):
	rclpy.init(args=args)
	vx_tgt_node = VelocityTgtNode()
	rclpy.spin(vx_tgt_node)
	vx_tgt_node.destroy_node()
	rclpy.shutdown

if __name__ =='__main__':
	main()

