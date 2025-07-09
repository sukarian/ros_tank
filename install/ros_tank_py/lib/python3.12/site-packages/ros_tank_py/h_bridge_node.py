import rclpy
import serial, time
from rclpy.node import Node
import lgpio
import fcntl

from std_msgs.msg import Int32

class BridgeNode(Node):

    def __init__(self):
        super().__init__('h_bridge_node')
        self.right_serial_cmd_sub = self.create_subscription(Int32, 'serial/motor_cmd/right', self.right_cmd_callback, 10)
        self.left_serial_cmd_sub = self.create_subscription(Int32, 'serial/motor_cmd/left', self.left_cmd_callback, 10)
        self.right_speed = Int32()
        self.left_speed = Int32()
        self.mutex_file = open('/tmp/gpio_mutex', 'w')
        #self.deadband = self.get_parameter('deadband')
        self.enA = 18
        self.in1 = 23
        self.in2 = 24
        self.in3 = 25
        self.in4 = 12
        self.enB = 13
        fcntl.flock(self.mutex_file, fcntl.LOCK_EX)

        try:
            self.chip = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(self.chip, self.enA)
            lgpio.gpio_claim_output(self.chip, self.in1)
            lgpio.gpio_claim_output(self.chip, self.in2)
            lgpio.gpio_claim_output(self.chip, self.in3)
            lgpio.gpio_claim_output(self.chip, self.in4)
            lgpio.gpio_claim_output(self.chip, self.enB)
        finally:
            fcntl.flock(self.mutex_file, fcntl.LOCK_UN)

        self.timer_period = 0.005  # seconds
        self.duty_period = self.timer_period * 10
        self.left_time = time.time()
        self.right_time = time.time()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        time.sleep(1)
        
    def right_cmd_callback(self, msg):
        self.right_speed = msg
        
    def left_cmd_callback(self, msg):
        self.left_speed = msg
        
    def timer_callback(self):
        self.move_motors()
    
    def move_motors(self):
        if self.left_speed.data > 0:
            self.left_forward()
        elif self.left_speed.data < 0:
            self.left_backward()
        else:
            self.left_stop()
            
        if self.right_speed.data > 0:
            self.right_forward()
        elif self.right_speed.data < 0:
            self.right_backward()
        else:
            self.right_stop()
            
        duty_cycle_right = abs(self.right_speed.data)/250 * (self.timer_period * 10)
        duty_cycle_left = abs(self.left_speed.data)/250 * (self.timer_period * 10)
        
        if duty_cycle_right > duty_cycle_left:
            self.pwm(self.enA, self.enB, duty_cycle_right, duty_cycle_left)
        else:
            self.pwm(self.enB, self.enA, duty_cycle_left, duty_cycle_right)

    def left_stop(self):
        fcntl.flock(self.mutex_file, fcntl.LOCK_EX)
        try:
            lgpio.gpio_write(self.chip, self.in3, 0)
            lgpio.gpio_write(self.chip, self.in4, 0)
        finally:
            fcntl.flock(self.mutex_file, fcntl.LOCK_UN)

    def left_forward(self):
        fcntl.flock(self.mutex_file, fcntl.LOCK_EX)
        try:
            lgpio.gpio_write(self.chip, self.in3, 1)
            lgpio.gpio_write(self.chip, self.in4, 0)
        finally:
            fcntl.flock(self.mutex_file, fcntl.LOCK_UN)

    def left_backward(self):
        fcntl.flock(self.mutex_file, fcntl.LOCK_EX)
        try:
            lgpio.gpio_write(self.chip, self.in3, 0)
            lgpio.gpio_write(self.chip, self.in4, 1)
        finally:
            fcntl.flock(self.mutex_file, fcntl.LOCK_UN)

    def right_stop(self):
        fcntl.flock(self.mutex_file, fcntl.LOCK_EX)
        try:
            lgpio.gpio_write(self.chip, self.in1, 0)
            lgpio.gpio_write(self.chip, self.in2, 0)
        finally:
            fcntl.flock(self.mutex_file, fcntl.LOCK_UN)

    def right_forward(self):
        fcntl.flock(self.mutex_file, fcntl.LOCK_EX)
        try:
            lgpio.gpio_write(self.chip, self.in1, 1)
            lgpio.gpio_write(self.chip, self.in2, 0)
        finally:
            fcntl.flock(self.mutex_file, fcntl.LOCK_UN)

    def right_backward(self):
        fcntl.flock(self.mutex_file, fcntl.LOCK_EX)
        try:
            lgpio.gpio_write(self.chip, self.in1, 0)
            lgpio.gpio_write(self.chip, self.in2, 1)
        finally:
            fcntl.flock(self.mutex_file, fcntl.LOCK_UN)

    def pwm(self, en1, en2, duty_cycle1, duty_cycle2):
        duty_cycle_diff = duty_cycle1 - duty_cycle2
        fcntl.flock(self.mutex_file, fcntl.LOCK_EX)
        try:
            lgpio.gpio_write(self.chip, en1, 1)
            lgpio.gpio_write(self.chip, en2, 1)
            time.sleep(duty_cycle2)
            lgpio.gpio_write(self.chip, en2, 0)
            time.sleep(duty_cycle_diff)
            lgpio.gpio_write(self.chip, en1, 0)
        finally:
            fcntl.flock(self.mutex_file, fcntl.LOCK_UN)
        time.sleep(self.duty_period - duty_cycle1)
        
        
def main(args=None):
    rclpy.init(args=args)
    h_bridge_node = BridgeNode()
    try:
        while True:
            rclpy.spin(h_bridge_node)
    except KeyboardInterrupt:
        h_bridge_node.destroy_node()
        h_bridge_node.left_stop()
        h_bridge_node.right_stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
