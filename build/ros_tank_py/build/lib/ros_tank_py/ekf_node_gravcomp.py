import rclpy
import serial, time
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose
import numpy as np
from scipy.linalg import expm

class ExtendedKalmanFilter:
	def __init__(self):
		# State vector: [x, y, theta, v, omega, ax, ay, roll, pitch, r, p]
		self.state = np.zeros(11)
		self.P = np.eye(11)
		self.Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
		self.R_imu = np.diag([0.00329, 0.00162, 2.2338723812647986e-06, 
			2.3823643044295863e-06, 4.493655223399326e-06])
		self.R_twist = np.diag([0.1, 0.1])

	def predict(self, dt):
		F = np.eye(11)
		F[0, 3] = dt * np.cos(self.state[2])
		F[1, 3] = dt * np.sin(self.state[2])
		F[2, 4] = dt # Angular velocity update from angular acceleration (not used here)
		F[3, 5] = dt # Velocity update from linear acceleration
		F[4, 6] = dt
		#F[5, 7] = dt
		#F[6, 8] = dt
		F[7, 9] = dt
		F[8, 10] = dt

		# Process noise
		G = np.zeros((11,11))
		G[3, 0] = dt # Linear acceleration noise
		G[4, 1] = dt # Angular acceleration noise


		self.state = F @ self.state #- grav_comp
		self.P = F @ self.P @ F.T + G @ self.Q @ G.T

	def update_imu(self, linear_acceleration, angular_velocity, imu_bias):
		H = np.array([[0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
			[0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]])

		z = np.array([linear_acceleration[0], linear_acceleration[1],
			angular_velocity[0], angular_velocity[1], angular_velocity[2]])

		grav_comp = np.zeros(5)
		x_term = -9.81 * np.sin(self.state[8])
		y_term = 9.81 * np.cos(self.state[8]) * np.sin(self.state[7])

		grav_comp[0] = x_term
		grav_comp[1] = y_term

		y = z - imu_bias - H @ self.state

		S = H @ self.P @ H.T + self.R_imu
		K = self.P @ H.T @ np.linalg.inv(S)

		self.state = self.state + K @ y

		self.P = (np.eye(11) - K @ H) @ self.P

	def update_twist(self, linear_velocity, angular_velocity):
		H = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]])

		z = np.array([linear_velocity, angular_velocity])

		y = z - H @ self.state

		S = H @ self.P @ H.T + self.R_twist
		K = self.P @ H.T @ np.linalg.inv(S)

		self.state = self.state + K @ y

		self.P = (np.eye(11) - K @ H) @ self.P

	def get_pose(self):
		return self.state[:3]

class EKFPoseEstimator(Node):
	def __init__(self):
		super().__init__('ekf_pose_estimator')
		self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
		self.twist_sub = self.create_subscription(Twist, 'twist/encoder', self.twist_callback, 10)
		self.pose_pub = self.create_publisher(Pose, '/estimated_pose', 10)
		self.ekf = ExtendedKalmanFilter()
		self.timer = self.create_timer(0.01, self.timer_callback)
		self.last_time = self.get_clock().now()
		self.declare_parameters(namespace='',
		parameters=[('x_bias',-0.4117),('y_bias',-0.0967),('z_bias',10.1881),
		('roll_bias',-0.0420),('pitch_bias',-0.0078),('yaw_bias',-0.075)])

		x_bias: double = self.get_parameter('x_bias').get_parameter_value().double_value
		y_bias: double = self.get_parameter('y_bias').get_parameter_value().double_value
		roll_bias: double = self.get_parameter('roll_bias').get_parameter_value().double_value
		pitch_bias: double = self.get_parameter('pitch_bias').get_parameter_value().double_value
		yaw_bias: double = self.get_parameter('yaw_bias').get_parameter_value().double_value
		self.imu_bias = [x_bias, y_bias, roll_bias, pitch_bias, yaw_bias]

	def imu_callback(self, msg):
		linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y]
		angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
		self.ekf.update_imu(linear_acceleration, angular_velocity, self.imu_bias)

	def twist_callback(self, msg):
		linear_velocity = msg.linear.x
		angular_velocity = msg.angular.z
		self.ekf.update_twist(linear_velocity, angular_velocity)

	def timer_callback(self):
		current_time = self.get_clock().now()
		dt = (current_time - self.last_time).nanoseconds/1e9
		self.last_time = current_time

		self.ekf.predict(dt)
		#self.get_logger().info(f"{np.degrees(self.ekf.state[7])}")
		#self.get_logger().info(f"Yaw: {np.degrees(self.ekf.state[8])}")
		#self.get_logger().info(f"-------------------")
		#self.get_logger().info(f"{np.degrees(self.ekf.state[9])}")
		#self.get_logger().info(f"{np.degrees(self.ekf.state[10])}")
		#self.get_logger().info(f"====================")

		pose_state = self.ekf.get_pose()

		pose_msg = Pose()
		pose_msg.position.x = pose_state[0]
		pose_msg.position.y = pose_state[1]
		pose_msg.orientation.z = np.sin(pose_state[2] / 2)
		pose_msg.orientation.w = np.cos(pose_state[2] / 2)
		self.pose_pub.publish(pose_msg)

def main(args=None):
	rclpy.init(args=args)
	node = EKFPoseEstimator()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()


