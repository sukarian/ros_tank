import rclpy
import serial, time
from rclpy.node import Node

from std_msgs.msg import Int32,Float32

class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')
        self.right_publisher_ = self.create_publisher(Int32, 'encoder/right', 10)
        self.left_publisher_ = self.create_publisher(Int32, 'encoder/left', 10)
        self.right_msg = Int32()
        self.left_msg = Int32()
        self.right_wheel_sub_ = self.create_subscription(Int32, 'serial/motor_cmd/right', self.right_cmd_callback, 10)
        self.left_wheel_sub_ = self.create_subscription(Int32, 'serial/motor_cmd/left', self.left_cmd_callback, 10)
        self.right_motor_cmd = Int32()
        self.left_motor_cmd = Int32()
        self.handshake = True
        self.right_rxd = False
        self.left_rxd = False
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pub_string = ''
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 0.1)
        self.ser.reset_input_buffer()
        time.sleep(1)
        
    def right_cmd_callback(self, msg):
        self.right_motor_cmd = msg
        self.right_rxd = True
        
    def left_cmd_callback(self, msg):
        self.left_motor_cmd = msg
        self.left_rxd = True

    def timer_callback(self):
        self.read_serial()
        #if (self.right_rxd and self.left_rxd):
        self.pub_string = str(self.left_motor_cmd.data) + ',' + str(self.right_motor_cmd.data)
        self.ser.write(bytes(self.pub_string,'utf-8'))
        #time.sleep(0.5)
        if (self.handshake == True):
            self.right_publisher_.publish(self.right_msg)
            self.left_publisher_.publish(self.left_msg)
        #    self.handshake = False

    def read_serial(self):
        try:
            line = self.ser.readline().decode("utf-8").rstrip()
            print("Line: " + str(line))
            if (line == 'RXD'):
                self.handshake = True
            elif (len(line.split(':')) > 1):
                msg = str(line.split(':')[1])
                if (line[0] == '1'):
                    self.left_msg.data = int(msg)
                elif (line[0] == '2'):
                    self.right_msg.data = int(msg)
                else:
                    print("Invalid message: " + str(line))
            else:
                print("Invalid line: " + str(line))
        except UnicodeDecodeError:
            return

        

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
   #while(True):

        #serial_node.ser.write(bytes(serial_node.pub_string,'utf-8'))
    #    rclpy.spin_once(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
