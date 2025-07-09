
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import readchar


class KeyboardNode(Node):

	def __init__(self):
		super().__init__('keyboard_node')
		self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
		self.twist = Twist()
		self.direction = " "
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def read_input(self):
		self.direction = readchar.readkey()
		fwd = self.twist.linear.x
		side = self.twist.angular.z
		fwd_inc = 0.1
		side_inc = 1
		if self.direction == "w":
			print("Key: " + self.direction)
			fwd += fwd_inc
		elif self.direction == "s":
			print("Key: " + self.direction)
			fwd -= fwd_inc
		elif self.direction == "q":
			print("Key: " + self.direction)
			fwd = 0
		elif self.direction == "d":
			print("Key: " + self.direction)
			side += side_inc
		elif self.direction == "a":
			print("Key: " + self.direction)
			side -= side_inc
		elif self.direction == "e":
			print("Key: " + self.direction)
			side = 0
		side = self.constrain(side, 30)
		fwd = self.constrain(fwd, 3)
		self.twist.linear.x = fwd*1.0
		self.twist.angular.z = side*1.0

	def constrain(self, dir, lim):
		if dir > lim:
			dir = lim
		elif dir < -1 * lim:
			dir = -1 * lim
		return dir

	def timer_callback(self):
		self.read_input()
		self.twist_pub.publish(self.twist)

def main(args=None):
	rclpy.init(args=args)
	keyboard_node = KeyboardNode()
	rclpy.spin(keyboard_node)
	keyboard_node.destroy_node()
	rclpy_shutdown()

if __name__ == '__main__':
	main()
