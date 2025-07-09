import rclpy
import serial, time
from rclpy.node import Node

from std_msgs.msg import Int32


class EncoderPublisher(Node):

    def __init__(self):
        super().__init__('encoder_publisher')
        self.right_publisher_ = self.create_publisher(Int32, 'encoder/right', 10)
        self.left_publisher_ = self.create_publisher(Int32, 'encoder/left', 10)
        self.right_msg = Int32()
        self.left_msg = Int32()
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 0.1)
        self.ser.reset_input_buffer()
        time.sleep(1)

    def timer_callback(self):
        self.read_serial()
        self.right_publisher_.publish(self.right_msg)
        self.left_publisher_.publish(self.left_msg)
        
    def data_pub(self):
        self.read_serial()
        self.right_publisher_.publish(self.right_msg)
        self.left_publisher_.publish(self.left_msg)

    def read_serial(self):
        try:
            line = self.ser.readline().decode("utf-8").rstrip()
            #print("Line: " + str(line))
            line_list = line.split(' ')
            if (len(line_list) > 1):
                self.left_msg.data = int(line_list[0])
                self.right_msg.data = int(line_list[1])
        except (UnicodeDecodeError, ValueError):
            return
        

def main(args=None):
    rclpy.init(args=args)
    encoder_publisher = EncoderPublisher()
    #while True:
    #encoder_publisher.data_pub()
    rclpy.spin(encoder_publisher)
    encoder_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
