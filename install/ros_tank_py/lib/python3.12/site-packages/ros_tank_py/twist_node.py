import rclpy
import serial, time
from rclpy.node import Node
import math

from std_msgs.msg import Float32,Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class TwistNode(Node):

    def __init__(self):
        super().__init__('twist_node')
        self.left_encoder_sub = self.create_subscription(Int32, 'encoder/left', self.left_encoder_callback, 10)
        self.right_encoder_sub = self.create_subscription(Int32, 'encoder/right', self.right_encoder_callback, 10)
        self.twist_pub_ = self.create_publisher(Twist, 'twist/encoder', 10)
        self.right_speed_pub_ = self.create_publisher(Float32, 'wheel_speed/right', 10)
        self.left_speed_pub_ = self.create_publisher(Float32, 'wheel_speed/left', 10)
        self.twist = Twist()
        self.right_encoder = Int32()
        self.left_encoder = Int32()
        self.right_speed = Float32()
        self.left_speed = Float32()
        self.left_time = 0
        self.left_dt = 0
        self.right_time = 0
        self.right_dt = 0
        self.last_right_encoder = 0
        self.last_left_encoder = 0
        self.first_cycle_left = True
        self.first_cycle_right = True
        self.left_ready = False
        self.right_ready = False
        self.left_processed = False
        self.right_processed = False
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def left_encoder_callback(self, msg):
        self.left_encoder = msg
        if self.first_cycle_left:
            self.last_left_encoder = msg.data
            self.first_cycle_left = False
    
    def right_encoder_callback(self, msg):
        self.right_encoder = msg
        if self.first_cycle_right:
            self.last_right_encoder = msg.data
            self.first_cycle_right = False
        
    def timer_callback(self):
        self.left_process()
        self.right_process()
        self.compute_twist()
        self.twist_pub_.publish(self.twist)
        self.right_speed_pub_.publish(self.right_speed)
        self.left_speed_pub_.publish(self.left_speed)

    def left_process(self):
        left_encoder_m = self.convert_data(self.left_encoder.data)
        self.left_dt = time.time() - self.left_time
        self.left_time = time.time()
        last_left_encoder_m = self.convert_data(self.last_left_encoder)
        if (self.last_left_encoder > 30000 and self.left_encoder.data < -30000) or (self.last_left_encoder < -30000 and self.left_encoder.data > 30000):
            self.left_speed.data = (last_left_encoder_m + left_encoder_m) / self.left_dt
        else:
            self.left_speed.data = (left_encoder_m - last_left_encoder_m) / self.left_dt
        self.last_left_encoder = self.left_encoder.data
        self.left_ready = False
        
    def right_process(self):
        right_encoder_m = self.convert_data(self.right_encoder.data)
        self.right_dt = time.time() - self.right_time
        self.right_time = time.time()
        last_right_encoder_m = self.convert_data(self.last_right_encoder)
        if (self.last_right_encoder > 30000 and self.right_encoder.data < -30000) or (self.last_right_encoder < -30000 and self.right_encoder.data > 30000):
            self.right_speed.data = (last_right_encoder_m + right_encoder_m) / self.right_dt
        else:
            self.right_speed.data = (right_encoder_m - last_right_encoder_m) / self.right_dt
        self.last_right_encoder = self.right_encoder.data
        self.right_ready = False
        
    def convert_data(self, msg):
        return msg/600.0 * 0.05751 * math.pi
        
    def compute_twist(self):
        self.twist.linear.x = (self.right_speed.data + self.left_speed.data)/2
        self.twist.angular.z = (self.right_speed.data - self.left_speed.data)/0.155

        
def main(args=None):
    rclpy.init(args=args)
    twist_node = TwistNode()
    rclpy.spin(twist_node)
    twist_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
