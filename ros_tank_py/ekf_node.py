
import rclpy
import serial, time
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose
import numpy as np
from scipy.spatial.transform import Rotation

class ExtendedKalmanFilter:
    def __init__(self):
        # State vector: [x, y, theta, vx, vy, omega, ax, ay, roll, pitch]
        self.state = np.zeros(10)
        
        # Covariance matrix
        self.P = np.eye(10) * 0.1
        
        # Process noise (tuned for better performance)
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.01, 0.01, 0.1, 0.1])
        
        # IMU measurement noise
        self.R_imu = np.diag([0.00329, 0.00162, 2.2338723812647986e-06,
                2.3823643044295863e-06, 4.493655223399326e-06])
        
        # Twist measurement noise
        self.R_twist = np.diag([0.1, 0.1])
        
        # Gravity constant
        self.g = 9.81

        #amount of cycles
        self.init_cycles = 50

    def predict(self, dt):
        # Extract current state
        x, y, theta, vx, vy, omega, ax, ay, roll, pitch = self.state
        
        # Create state transition matrix F
        F = np.eye(10)
        
        # Position updates from velocity (in global frame)
        F[0, 3] = dt  # x = x + vx * dt
        F[1, 4] = dt  # y = y + vy * dt
        
        # Orientation update from angular velocity
        F[2, 5] = dt  # theta = theta + omega * dt
        
        # Velocity updates from acceleration (in global frame)
        F[3, 6] = dt  # vx = vx + ax * dt
        F[4, 7] = dt  # vy = vy + ay * dt
        
        # Predict next state
        self.state = F @ self.state
        
        # Normalize angle
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Process noise matrix
        G = np.eye(10)
        
        # Update covariance
        self.P = F @ self.P @ F.T + G @ self.Q @ G.T

    def update_imu(self, linear_acceleration, angular_velocity, imu_bias):
        # Measurement model for IMU
        # IMU measures: [ax_body, ay_body, roll_rate, pitch_rate, yaw_rate]
        H = np.zeros((5, 10))
        H[0, 6] = 1  # ax measurement
        H[1, 7] = 1  # ay measurement
        H[2, 8] = 1  # roll rate (approximate)
        H[3, 9] = 1  # pitch rate (approximate)
        H[4, 5] = 1  # yaw rate (omega)

        theta, roll, pitch = self.state[2], self.state[8], self.state[9]
        
        ax_body = linear_acceleration[0] - imu_bias[0]
        ay_body = linear_acceleration[1] - imu_bias[1]
        expected_ax = ax_body - self.g * np.sin(pitch)
        expected_ay = ay_body + self.g * np.cos(pitch) * np.sin(roll)
    
        # Transform to global frame
        ax_global = expected_ax * np.cos(theta) - expected_ay * np.sin(theta)
        ay_global = expected_ax * np.sin(theta) + expected_ay * np.cos(theta)
    
        # Store global frame accelerations
        self.state[6] = ax_global
        self.state[7] = ay_global
        
        # Expected accelerations in body frame (including gravity)
        h_x = np.array([
            ax_global,
            ay_global,
            self.state[8],  # roll rate
            self.state[9],  # pitch rate
            self.state[5]   # yaw rate
        ])
        
        # Actual measurements
        z = np.array([
            linear_acceleration[0],
            linear_acceleration[1],
            angular_velocity[0],
            angular_velocity[1],
            angular_velocity[2]
        ])
        
        # Remove bias
        z_corrected = z - imu_bias
        
        # Innovation
        y = z_corrected - h_x
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_imu
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state = self.state + K @ y

        # Normalize angle
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Covariance update
        self.P = (np.eye(10) - K @ H) @ self.P

    def update_twist(self, linear_velocity, angular_velocity):
        # Measurement model for encoder/twist data
        # Assumes twist is in body frame, convert to global frame
        theta = self.state[2]
        
        # Convert body frame velocity to global frame
        vx_global = linear_velocity * np.cos(theta)
        vy_global = linear_velocity * np.sin(theta)
        
        H = np.zeros((2, 10))
        H[0, 3] = 1  # vx measurement
        H[1, 4] = 1  # vy measurement
        
        z = np.array([vx_global, vy_global])
        
        # Innovation
        y = z - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_twist
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state = self.state + K @ y
        
        # Covariance update
        self.P = (np.eye(10) - K @ H) @ self.P

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def get_pose(self):
        return self.state[:3]  # [x, y, theta]
    
    def get_full_state(self):
        return self.state.copy()

class EKFPoseEstimator(Node):
    def __init__(self):
        super().__init__('ekf_pose_estimator')
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.twist_sub = self.create_subscription(Twist, 'twist/encoder', self.twist_callback, 10)
        
        # Publishers
        self.pose_pub = self.create_publisher(Pose, '/estimated_pose', 10)
        
        # EKF instance
        self.ekf = ExtendedKalmanFilter()
        
        # Timer for prediction step
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.last_time = self.get_clock().now()
        
        # IMU bias parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('x_bias', -0.4117),
                ('y_bias', -0.0967), 
                ('z_bias', 10.1881),
                ('roll_bias', -0.0420),
                ('pitch_bias', -0.0078),
                ('yaw_bias', -0.0753)
            ])

        # Get bias values
        x_bias = self.get_parameter('x_bias').get_parameter_value().double_value
        y_bias = self.get_parameter('y_bias').get_parameter_value().double_value
        roll_bias = self.get_parameter('roll_bias').get_parameter_value().double_value
        pitch_bias = self.get_parameter('pitch_bias').get_parameter_value().double_value
        yaw_bias = self.get_parameter('yaw_bias').get_parameter_value().double_value
        
        self.imu_bias = [x_bias, y_bias, roll_bias, pitch_bias, yaw_bias]
        
        self.num_cycles = 0
        self.init_cycles = 300
        self.get_logger().info("EKF Pose Estimator initialized")

    def imu_callback(self, msg):
        # Extract linear acceleration and angular velocity
        linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y]
        angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        
        # Update EKF with IMU data
        self.ekf.update_imu(linear_acceleration, angular_velocity, self.imu_bias)

    def twist_callback(self, msg):
        # Extract velocities
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        # Update EKF with twist data
        self.ekf.update_twist(linear_velocity, angular_velocity)

    def timer_callback(self):
        # Calculate time step
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Prediction step
        self.ekf.predict(dt)
        
        # Get estimated pose
        pose_state = self.ekf.get_pose()
        
        # Create and publish pose message
        pose_msg = Pose()
        pose_msg.position.x = pose_state[0]
        pose_msg.position.y = pose_state[1]
        pose_msg.position.z = 0.0  # Assuming 2D motion
        
        # Convert yaw to quaternion
        yaw = pose_state[2]
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = np.sin(yaw / 2.0)
        pose_msg.orientation.w = np.cos(yaw / 2.0)
        
        self.num_cycles += 1

        # Publish pose
        if self.num_cycles > self.init_cycles:
            self.pose_pub.publish(pose_msg)
        
        # Optional: Log pose for debugging
        self.get_logger().info(f"Pose: x={pose_state[0]:.3f}, y={pose_state[1]:.3f}, theta={np.degrees(pose_state[2]):.1f}°")

def main(args=None):
    rclpy.init(args=args)
    node = EKFPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
