import rclpy
import serial, time
from rclpy.node import Node

from std_msgs.msg import Float32,Int32

class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')
        self.right_wheel_sub = self.create_subscription(Float32, 'wheel_speed/right', self.right_feedback_callback, 10)
        self.left_wheel_sub = self.create_subscription(Float32, 'wheel_speed/left', self.left_feedback_callback, 10)
        self.right_motor_cmd_sub = self.create_subscription(Float32, 'motor_cmd/right', self.right_cmd_callback, 10)
        self.left_motor_cmd_sub = self.create_subscription(Float32, 'motor_cmd/left', self.left_cmd_callback, 10)
        self.right_serial_cmd_pub = self.create_publisher(Int32, 'serial/motor_cmd/right', 10)
        self.left_serial_cmd_pub = self.create_publisher(Int32, 'serial/motor_cmd/left', 10)
        self.right_motor_cmd = Float32()
        self.left_motor_cmd = Float32()
        self.right_wheel = Float32()
        self.left_wheel = Float32()
        self.right_serial_cmd = Int32() 
        self.left_serial_cmd = Int32()

        self.declare_parameters(namespace='',
        parameters=[('Kp',1),('Ki',1),('Kd',1),('deadband',1)])

        self.left_Kp: int = self.get_parameter('Kp').get_parameter_value().integer_value
        self.left_Ki: int = self.get_parameter('Ki').get_parameter_value().integer_value
        self.left_Kd: int = self.get_parameter('Kd').get_parameter_value().integer_value
        self.left_error_int = 0
        self.left_error_last = 0
        self.left_dt = 0
        self.left_time = time.time()
        self.right_Kp: int  = self.get_parameter('Kp').get_parameter_value().integer_value
        self.right_Ki: int = self.get_parameter('Ki').get_parameter_value().integer_value
        self.right_Kd: int = self.get_parameter('Kd').get_parameter_value().integer_value
        print(self.right_Kp)
        self.deadband = self.get_parameter('deadband').get_parameter_value().integer_value
        self.right_error_int = 0
        self.right_error_last = 0
        self.right_dt = 0
        self.right_time = time.time()
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        time.sleep(1)
        
    def right_feedback_callback(self, msg):
        self.right_wheel = msg
        
    def left_feedback_callback(self, msg):
        self.left_wheel = msg
        
    def right_cmd_callback(self, msg):
        self.right_motor_cmd = msg
        
    def left_cmd_callback(self, msg):
        self.left_motor_cmd = msg
        
    def timer_callback(self):
        self.right_PID()
        self.left_PID()
        
    def right_PID(self):
        self.right_dt = time.time() - self.right_time
        self.right_time = time.time()
        speed_error = self.right_motor_cmd.data - self.right_wheel.data
        self.right_error_last = speed_error
        self.right_error_int += speed_error*self.right_dt 
        d_error = (speed_error - self.right_error_last) / self.right_dt
        self.right_serial_cmd.data = int(self.right_Kp*speed_error + self.right_Ki*self.right_error_int + self.right_Kd*d_error)
        self.right_serial_cmd.data = self.constrain(self.right_serial_cmd.data, -250, 250)
        self.right_serial_cmd_pub.publish(self.right_serial_cmd)
        
    def left_PID(self):
        self.left_dt = time.time() - self.left_time
        self.left_time = time.time()
        speed_error = self.left_motor_cmd.data - self.left_wheel.data
        #print(self.left_wheel.data)
        #print(speed_error)
        self.left_error_last = speed_error
        self.left_error_int += speed_error*self.left_dt 
        d_error = (speed_error - self.left_error_last) / self.left_dt
        self.left_serial_cmd.data = int(self.left_Kp*speed_error + self.left_Ki*self.left_error_int + self.left_Kd*d_error)
        self.left_serial_cmd.data = self.constrain(self.left_serial_cmd.data, -250, 250)
        self.left_serial_cmd_pub.publish(self.left_serial_cmd)
        
    def constrain(self, val, min_val, max_val):
        out_val = min(max_val, max(min_val, val))
        if abs(out_val) < self.deadband:
                out_val = 0
        return out_val

    def __del__(self):
        lgpio.cleanup()
        
def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
