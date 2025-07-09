import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu

class BiasNode(Node):
	def __init__(self):
		super().__init__('bias_node')
		self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
		self.imu_msg = Imu()
		self.num_msgs = 1
		self.sum_x = 0
		self.sum_y = 0
		self.sum_z = 0
		self.sum_roll = 0
		self.sum_pitch = 0
		self.sum_yaw = 0
		self.avg_x = 0
		self.avg_y = 0
		self.avg_z = 0
		self.avg_roll = 0
		self.avg_pitch = 0
		self.avg_yaw = 0
		self.linear_acceleration_data = []
		self.angular_velocity_data = []
		self.timer = self.create_timer(0.1, self.timer_callback)

	def imu_callback(self, msg):
		self.imu_msg = msg

	def calc_biases(self):
		self.linear_acceleration_data.append([self.imu_msg.linear_acceleration.x,
			self.imu_msg.linear_acceleration.y,
			self.imu_msg.linear_acceleration.z])
		self.angular_velocity_data.append([self.imu_msg.angular_velocity.x,
			self.imu_msg.angular_velocity.y,
			self.imu_msg.angular_velocity.z])
		self.sum_x += self.imu_msg.linear_acceleration.x
		self.sum_y += self.imu_msg.linear_acceleration.y
		self.sum_z += self.imu_msg.linear_acceleration.z

		self.avg_x = self.sum_x / self.num_msgs
		self.avg_y = self.sum_y / self.num_msgs
		self.avg_z = self.sum_z / self.num_msgs

		self.sum_roll += self.imu_msg.angular_velocity.x
		self.sum_pitch += self.imu_msg.angular_velocity.y
		self.sum_yaw += self.imu_msg.angular_velocity.z

		self.avg_roll = self.sum_roll / self.num_msgs
		self.avg_pitch = self.sum_pitch / self.num_msgs
		self.avg_yaw = self.sum_yaw / self.num_msgs
		self.num_msgs += 1

	def timer_callback(self):
		if self.num_msgs < 1000:
			self.calc_biases()
		else:
			self.comp_covar()
		print('-----------')
		print(self.avg_x)
		print(self.avg_y)
		print(self.avg_z)
		print(self.avg_roll)
		print(self.avg_pitch)
		print(self.avg_yaw)
		print('-----------')

	def comp_covar(self):
		linear_acceleration_data = np.array(self.linear_acceleration_data)
		angular_velocity_data = np.array(self.angular_velocity_data)

		# Compute covariance matrices
		#linear_acceleration_covariance = np.var(linear_acceleration_data) #, rowvar=False)
		#angular_velocity_covariance = np.var(angular_velocity_data) #, rowvar=False)
		x_acceleration_var = np.var(linear_acceleration_data[:,0]) #, rowvar=False)
		y_acceleration_var = np.var(linear_acceleration_data[:,1]) #, rowvar=False)
		z_acceleration_var = np.var(linear_acceleration_data[:,2]) #, rowvar=False)

		x_vel_var = np.var(angular_velocity_data[:,0]) #, rowvar=False)
		y_vel_var = np.var(angular_velocity_data[:,1]) #, rowvar=False)
		z_vel_var = np.var(angular_velocity_data[:,2]) #, rowvar=False)

		# Log the results
		print("Linear Acceleration Covariance Matrix:")
		print(x_acceleration_var)
		print(y_acceleration_var)
		print(z_acceleration_var)
		print("Angular Velocity Covariance Matrix:")
		print(x_vel_var)
		print(y_vel_var)
		print(z_vel_var)

def main(args=None):
	rclpy.init(args=args)
	node = BiasNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy_shutdown()

if __name__ == '__main__':
	main()
