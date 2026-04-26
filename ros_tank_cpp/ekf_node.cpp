#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <memory>

class ExtendedKalmanFilter {
private:
    static constexpr double PI = M_PI;
    
public:
    ExtendedKalmanFilter() {
        // State vector: [x, y, theta, vx, vy, omega, ax, ay, roll, pitch]
        state_ = Eigen::VectorXd::Zero(10);
        
        // Covariance matrix
        P_ = Eigen::MatrixXd::Identity(10, 10) * 0.1;
        
        // Process noise (tuned for better performance)
        Q_ = Eigen::MatrixXd::Zero(10, 10);
        Q_.diagonal() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.1, 0.1;
        
        // IMU measurement noise
        R_imu_ = Eigen::MatrixXd::Zero(5, 5);
        R_imu_.diagonal() << 0.00329, 0.00162, 2.2338723812647986e-06,
                             2.3823643044295863e-06, 4.493655223399326e-06;
        
        // Twist measurement noise
        R_twist_ = Eigen::MatrixXd::Zero(2, 2);
        R_twist_.diagonal() << 0.1, 0.1;
        
        // Gravity constant
        g_ = 9.81;
        
        // Amount of cycles
        init_cycles_ = 50;
    }
    
    void predict(double dt) {
        // Extract current state
        double x = state_(0), y = state_(1), theta = state_(2);
        double vx = state_(3), vy = state_(4), omega = state_(5);
        double ax = state_(6), ay = state_(7), roll = state_(8), pitch = state_(9);
        
        // Create state transition matrix F
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(10, 10);
        
        // Position updates from velocity (in global frame)
        F(0, 3) = dt;  // x = x + vx * dt
        F(1, 4) = dt;  // y = y + vy * dt
        
        // Orientation update from angular velocity
        F(2, 5) = dt;  // theta = theta + omega * dt
        
        // Velocity updates from acceleration (in global frame)
        F(3, 6) = dt;  // vx = vx + ax * dt
        F(4, 7) = dt;  // vy = vy + ay * dt
        
        // Predict next state
        state_ = F * state_;
        
        // Normalize angle
        state_(2) = normalizeAngle(state_(2));
        
        // Process noise matrix
        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(10, 10);
        
        // Update covariance
        P_ = F * P_ * F.transpose() + G * Q_ * G.transpose();
    }
    
    void updateIMU(const std::vector<double>& linear_acceleration,
                   const std::vector<double>& angular_velocity,
                   const std::vector<double>& imu_bias) {
        // Measurement model for IMU
        // IMU measures: [ax_body, ay_body, roll_rate, pitch_rate, yaw_rate]
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(5, 10);
        H(0, 6) = 1;  // ax measurement
        H(1, 7) = 1;  // ay measurement
        H(2, 8) = 1;  // roll rate (approximate)
        H(3, 9) = 1;  // pitch rate (approximate)
        H(4, 5) = 1;  // yaw rate (omega)
        
        double theta = state_(2), roll = state_(8), pitch = state_(9);
        
        double ax_body = linear_acceleration[0] - imu_bias[0];
        double ay_body = linear_acceleration[1] - imu_bias[1];
        double expected_ax = ax_body - g_ * sin(pitch);
        double expected_ay = ay_body + g_ * cos(pitch) * sin(roll);
        
        // Transform to global frame
        double ax_global = expected_ax * cos(theta) - expected_ay * sin(theta);
        double ay_global = expected_ax * sin(theta) + expected_ay * cos(theta);
        
        // Store global frame accelerations
        state_(6) = ax_global;
        state_(7) = ay_global;
        
        // Expected accelerations in body frame (including gravity)
        Eigen::VectorXd h_x(5);
        h_x << ax_global,
               ay_global,
               state_(8),  // roll rate
               state_(9),  // pitch rate
               state_(5);  // yaw rate
        
        // Actual measurements
        Eigen::VectorXd z(5);
        z << linear_acceleration[0],
             linear_acceleration[1],
             angular_velocity[0],
             angular_velocity[1],
             angular_velocity[2];
        
        // Remove bias
        Eigen::VectorXd z_corrected(5);
        for (int i = 0; i < 5; ++i) {
            z_corrected(i) = z(i) - imu_bias[i];
        }
        
        // Innovation
        Eigen::VectorXd y = z_corrected - h_x;
        
        // Innovation covariance
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_imu_;
        
        // Kalman gain
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // State update
        state_ = state_ + K * y;
        
        // Normalize angle
        state_(2) = normalizeAngle(state_(2));
        
        // Covariance update
        P_ = (Eigen::MatrixXd::Identity(10, 10) - K * H) * P_;
    }
    
    void updateTwist(double linear_velocity, double angular_velocity) {
        // Measurement model for encoder/twist data
        // Assumes twist is in body frame, convert to global frame
        double theta = state_(2);
        
        // Convert body frame velocity to global frame
        double vx_global = linear_velocity * cos(theta);
        double vy_global = linear_velocity * sin(theta);
        
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 10);
        H(0, 3) = 1;  // vx measurement
        H(1, 4) = 1;  // vy measurement
        
        Eigen::VectorXd z(2);
        z << vx_global, vy_global;
        
        // Innovation
        Eigen::VectorXd y = z - H * state_;
        
        // Innovation covariance
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_twist_;
        
        // Kalman gain
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // State update
        state_ = state_ + K * y;
        
        // Covariance update
        P_ = (Eigen::MatrixXd::Identity(10, 10) - K * H) * P_;
    }
    
    double normalizeAngle(double angle) {
        while (angle > PI) {
            angle -= 2 * PI;
        }
        while (angle < -PI) {
            angle += 2 * PI;
        }
        return angle;
    }
    
    Eigen::Vector3d getPose() {
        return state_.head(3);  // [x, y, theta]
    }
    
    Eigen::VectorXd getFullState() {
        return state_;
    }
    
private:
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_imu_;
    Eigen::MatrixXd R_twist_;
    double g_;
    int init_cycles_;
};

class EKFPoseEstimator : public rclcpp::Node {
public:
    EKFPoseEstimator() : Node("ekf_pose_estimator"), num_cycles_(0), init_cycles_(300) {
        // Subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&EKFPoseEstimator::imuCallback, this, std::placeholders::_1));
        
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/twist/encoder", 10,
            std::bind(&EKFPoseEstimator::twistCallback, this, std::placeholders::_1));
        
        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/estimated_pose", 10);
        
        // EKF instance
        ekf_ = std::make_unique<ExtendedKalmanFilter>();
        
        // Timer for prediction step
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),  // 100 Hz
            std::bind(&EKFPoseEstimator::timerCallback, this));
        
        last_time_ = this->get_clock()->now();
        
        // IMU bias parameters
        this->declare_parameter("x_bias", -0.4117);
        this->declare_parameter("y_bias", -0.0967);
        this->declare_parameter("z_bias", 10.1881);
        this->declare_parameter("roll_bias", -0.0420);
        this->declare_parameter("pitch_bias", -0.0078);
        this->declare_parameter("yaw_bias", -0.0753);
        
        // Get bias values
        double x_bias = this->get_parameter("x_bias").as_double();
        double y_bias = this->get_parameter("y_bias").as_double();
        double roll_bias = this->get_parameter("roll_bias").as_double();
        double pitch_bias = this->get_parameter("pitch_bias").as_double();
        double yaw_bias = this->get_parameter("yaw_bias").as_double();
        
        imu_bias_ = {x_bias, y_bias, roll_bias, pitch_bias, yaw_bias};
        
        RCLCPP_INFO(this->get_logger(), "EKF Pose Estimator initialized");
    }
    
private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Extract linear acceleration and angular velocity
        std::vector<double> linear_acceleration = {
            msg->linear_acceleration.x,
            msg->linear_acceleration.y
        };
        std::vector<double> angular_velocity = {
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        };
        
        // Update EKF with IMU data
        ekf_->updateIMU(linear_acceleration, angular_velocity, imu_bias_);
    }
    
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Extract velocities
        double linear_velocity = msg->linear.x;
        double angular_velocity = msg->angular.z;
        
        // Update EKF with twist data
        ekf_->updateTwist(linear_velocity, angular_velocity);
    }
    
    void timerCallback() {
        // Calculate time step
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        
        // Prediction step
        ekf_->predict(dt);
        
        // Get estimated pose
        Eigen::Vector3d pose_state = ekf_->getPose();
        
        // Create and publish pose message
        auto pose_msg = geometry_msgs::msg::Pose();
        pose_msg.position.x = pose_state(0);
        pose_msg.position.y = pose_state(1);
        pose_msg.position.z = 0.0;  // Assuming 2D motion
        
        // Convert yaw to quaternion
        double yaw = pose_state(2);
        pose_msg.orientation.x = 0.0;
        pose_msg.orientation.y = 0.0;
        pose_msg.orientation.z = sin(yaw / 2.0);
        pose_msg.orientation.w = cos(yaw / 2.0);
        
        num_cycles_++;
        pose_pub_->publish(pose_msg);
        
        // Publish pose
        //if (num_cycles_ > init_cycles_) {
        //    pose_pub_->publish(pose_msg);
        //}
        
        // Optional: Log pose for debugging
        //RCLCPP_INFO(this->get_logger(), 
        //           "Pose: x=%.3f, y=%.3f, theta=%.1f°",
        //           pose_state(0), pose_state(1), pose_state(2) * 180.0 / M_PI);
    }
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::unique_ptr<ExtendedKalmanFilter> ekf_;
    rclcpp::Time last_time_;
    std::vector<double> imu_bias_;
    int num_cycles_;
    int init_cycles_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFPoseEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
