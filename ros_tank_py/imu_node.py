import rclpy
import time
from rclpy.node import Node
from smbus2 import SMBus
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
import fcntl

# MPU6050 Registers
IMU_ADDR = 0x68  # I2C address of MPU6050
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# MPU6050 Configuration
ACCEL_SCALE = 16384.0  # for ±2g range
GYRO_SCALE = 131.0     # for ±250°/s range

class ImuNode(Node):

	def __init__(self):
		super().__init__('imu_node')
		self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

		self.bus = SMBus(1)
		self.mutex_file = open('/tmp/gpio_mutex', 'w')
		#time.sleep(1)
		try:
			self.bus.write_byte_data(IMU_ADDR, PWR_MGMT_1, 0)
		finally:
			fcntl.flock(self.mutex_file, fcntl.LOCK_UN)
		self.timer = self.create_timer(0.1, self.timer_callback)

	def read_raw_data(self, addr):
		fcntl.flock(self.mutex_file, fcntl.LOCK_EX)
		try:
			high = self.bus.read_byte_data(IMU_ADDR, addr)
			low = self.bus.read_byte_data(IMU_ADDR, addr+1)

			data = (high << 8) | low

			if data > 32768:
				data -= 65536
			return data
		finally:
			fcntl.flock(self.mutex_file, fcntl.LOCK_UN)

	def read_imu_data(self):
		accel_x = self.read_raw_data(ACCEL_XOUT_H) / ACCEL_SCALE * -1
		accel_y = self.read_raw_data(ACCEL_XOUT_H + 2) / ACCEL_SCALE
		accel_z = self.read_raw_data(ACCEL_XOUT_H + 4) / ACCEL_SCALE

		gyro_x = self.read_raw_data(GYRO_XOUT_H) / GYRO_SCALE
		gyro_y = self.read_raw_data(GYRO_XOUT_H + 2) / GYRO_SCALE
		gyro_z = self.read_raw_data(GYRO_XOUT_H + 4) / GYRO_SCALE
		return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

	def timer_callback(self):
		accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_imu_data()

		imu_msg = Imu()

		imu_msg.header = Header()
		imu_msg.header.stamp = self.get_clock().now().to_msg()
		imu_msg.header.frame_id = 'imu_link'

		imu_msg.linear_acceleration.x = accel_x * 9.81
		imu_msg.linear_acceleration.y = accel_y * 9.81
		imu_msg.linear_acceleration.z = accel_z * 9.81

		imu_msg.angular_velocity.x = np.radians(gyro_x)
		imu_msg.angular_velocity.y = np.radians(gyro_y)
		imu_msg.angular_velocity.z = np.radians(gyro_z)

		self.imu_pub.publish(imu_msg)
		#self.get_logger().info(f"{imu_msg.angular_velocity.x}")
		#self.get_logger().info(f"{imu_msg.angular_velocity.y}")
		#self.get_logger().info(f"{imu_msg.angular_velocity.z}")
		#self.get_logger().info(f"-------------------")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


