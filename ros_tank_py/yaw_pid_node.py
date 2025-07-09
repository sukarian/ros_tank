import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist, Pose
from tf_transformations import euler_from_quaternion
import time
import math as m
import numpy as np

class YawPIDController(Node):
    def __init__(self):
        super().__init__('yaw_pid_control')
        self.create_subscription(Float32, '/yaw_tgt', self.yaw_callback, 10)
        self.create_subscription(Pose, '/estimated_pose', self.pose_callback, 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_done_sub = self.create_subscription(Bool, '/path_done', self.path_done_callback, 10)

        # Tuned PID gains - start conservative
        self.Kp = 100 # Proportional gain
        self.Ki = 0 #0.05   # Integral gain
        self.Kd = 0 #0.2    # Derivative gain
        
        # PID limits
        self.max_integral = 1.0  # Anti-windup
        self.max_output = 3.0    # Max yaw rate output
        
        # System state
        self.target_yaw = 0.0
        self.current_yaw = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
        self.start_time = time.time()
        
        # Status flags
        self.yaw_tgt_rxd = False
        self.pose_rxd = False
        self.path_done = False
        
        # Low-pass filter for derivative term
        self.derivative_filter_alpha = 0.2  # Smoothing factor
        self.filtered_derivative = 0.0
        
        self.timer = self.create_timer(0.01, self.timer_callback)

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        return (angle + m.pi) % (2 * m.pi) - m.pi

    def path_done_callback(self, msg):
        self.path_done = msg.data

    def yaw_callback(self, msg):
        self.target_yaw = self.normalize_angle(msg.data)
        self.yaw_tgt_rxd = True

    def pose_callback(self, msg):
        self.pose_rxd = True
        _, _, self.current_yaw = euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        self.current_yaw = self.normalize_angle(self.current_yaw)

    def control_loop(self):
        if not self.yaw_tgt_rxd or not self.pose_rxd:
            return
        
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0:
            return
            
        # Calculate error with angle wrapping
        error = self.normalize_angle(self.target_yaw - self.current_yaw)
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        I = self.Ki * self.integral
        
        # Derivative term with low-pass filtering
        raw_derivative = (error - self.prev_error) / dt
        self.filtered_derivative = (self.derivative_filter_alpha * raw_derivative + 
                                   (1 - self.derivative_filter_alpha) * self.filtered_derivative)
        D = self.Kd * self.filtered_derivative
        
        # Calculate output with saturation
        yaw_rate = P + I + D
        yaw_rate = np.clip(yaw_rate, -self.max_output, self.max_output)
        
        # Calculate forward velocity (optional)
        vx_magnitude = 0.25
        vx_tgt = vx_magnitude #* ((m.pi - abs(error)) / m.pi)
        
        # Publish command
        cmd_vel = Twist()
        cmd_vel.linear.x = vx_tgt
        cmd_vel.angular.z = yaw_rate

        if self.path_done:
           cmd_vel.linear.x = 0.0
           cmd_vel.angular.z = 0.0
           self.get_logger().info("Publishing zeros")
        self.twist_pub.publish(cmd_vel)
        # Update previous error
        self.prev_error = error
        
        # Logging
        #self.get_logger().info(
        #    f"Error: {error:.2f}, Yaw Rate: {yaw_rate:.2f}, "
        #    f"P: {P:.2f}, I: {I:.2f}, D: {D:.2f}"
        #)

    def timer_callback(self):
        self.control_loop()

def main(args=None):
    rclpy.init(args=args)
    node = YawPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
