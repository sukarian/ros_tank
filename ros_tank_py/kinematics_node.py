import rclpy
import serial, time
from rclpy.node import Node

from std_msgs.msg import Float32,Int32
from geometry_msgs.msg import Twist

class KinematicsNode(Node):

    def __init__(self):
        super().__init__('kinematics_node')
        self.twist_sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.right_wheel_pub_ = self.create_publisher(Float32, 'motor_cmd/right', 10)
        self.left_wheel_pub_ = self.create_publisher(Float32, 'motor_cmd/left', 10)
        self.twist = Twist()
        self.left_speed = Float32()
        self.right_speed = Float32()
        self.declare_parameters(namespace='',
        parameters=[('track_base',0.18)])
        self.track_base = self.get_parameter('track_base').get_parameter_value().double_value
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.left_speed.data = 0
        self.right_speed.data = 0
        self.left_speed.data += self.twist.linear.x
        self.right_speed.data += self.twist.linear.x
        self.left_speed.data -= self.twist.angular.z * self.track_base / 2
        self.right_speed.data += self.twist.angular.z * self.track_base / 2
        self.right_wheel_pub_.publish(self.right_speed)
        self.left_wheel_pub_.publish(self.left_speed)
        
    def twist_callback(self, msg):
        self.twist = msg
        
def main(args=None):
    rclpy.init(args=args)
    kinematics_node = KinematicsNode()
    rclpy.spin(kinematics_node)
    kinematics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
