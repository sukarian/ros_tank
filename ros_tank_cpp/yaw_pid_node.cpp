#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <cmath>
#include <algorithm>

class YawPIDController : public rclcpp::Node
{
public:
    YawPIDController() : Node("yaw_pid_control")
    {
        // Create subscriptions
        yaw_tgt_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/yaw_tgt", 10, std::bind(&YawPIDController::yaw_callback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/estimated_pose", 10, std::bind(&YawPIDController::pose_callback, this, std::placeholders::_1));
        
        path_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/path_done", 10, std::bind(&YawPIDController::path_done_callback, this, std::placeholders::_1));

        // Create publisher
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Tuned PID gains - start conservative
        Kp_ = 10.0;  // Proportional gain
        Ki_ = 0.0;    // Integral gain
        Kd_ = 0.0;    // Derivative gain

        // PID limits
        max_integral_ = 1.0;   // Anti-windup
        max_output_ = 3.0;     // Max yaw rate output

        // System state
        target_yaw_ = 0.0;
        current_yaw_ = 0.0;
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_time_ = std::chrono::steady_clock::now();
        start_time_ = std::chrono::steady_clock::now();

        // Status flags
        yaw_tgt_rxd_ = false;
        pose_rxd_ = false;
        path_done_ = false;

        // Low-pass filter for derivative term
        derivative_filter_alpha_ = 0.2;  // Smoothing factor
        filtered_derivative_ = 0.0;

        // Create timer (100 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&YawPIDController::timer_callback, this));
    }

private:
    // Normalize angle to [-π, π]
    double normalize_angle(double angle)
    {
        return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
    }

    void path_done_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        path_done_ = msg->data;
    }

    void yaw_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_yaw_ = normalize_angle(msg->data);
        yaw_tgt_rxd_ = true;
    }

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        pose_rxd_ = true;
        
        // Convert quaternion to euler angles
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        
        current_yaw_ = normalize_angle(yaw);
    }

    void control_loop()
    {
        if (!yaw_tgt_rxd_ || !pose_rxd_)
            return;

        auto current_time = std::chrono::steady_clock::now();
        auto dt_chrono = current_time - prev_time_;
        double dt = std::chrono::duration<double>(dt_chrono).count();
        prev_time_ = current_time;

        if (dt <= 0)
            return;

        // Calculate error with angle wrapping
        double error = normalize_angle(target_yaw_ - current_yaw_);

        // Proportional term
        double P = Kp_ * error;

        // Integral term with anti-windup
        integral_ += error * dt;
        integral_ = std::clamp(integral_, -max_integral_, max_integral_);
        double I = Ki_ * integral_;

        // Derivative term with low-pass filtering
        double raw_derivative = (error - prev_error_) / dt;
        filtered_derivative_ = (derivative_filter_alpha_ * raw_derivative +
                               (1 - derivative_filter_alpha_) * filtered_derivative_);
        double D = Kd_ * filtered_derivative_;

        // Calculate output with saturation
        double yaw_rate = P + I + D;
        yaw_rate = std::clamp(yaw_rate, -max_output_, max_output_);

        // Calculate forward velocity (optional)
        double vx_magnitude = 0.25;
        double vx_tgt = vx_magnitude * ((M_PI - std::abs(error)) / M_PI);

        // Publish command
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel.linear.x = vx_tgt;
        cmd_vel.angular.z = yaw_rate;

        if (path_done_)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Publishing zeros");
        }
        
        twist_pub_->publish(cmd_vel);
        
        // Update previous error
        prev_error_ = error;

        // Logging (commented out)
        // RCLCPP_INFO(this->get_logger(),
        //     "Error: %.2f, Yaw Rate: %.2f, P: %.2f, I: %.2f, D: %.2f",
        //     error, yaw_rate, P, I, D);
    }

    void timer_callback()
    {
        control_loop();
    }

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_tgt_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_done_sub_;
    
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // PID gains
    double Kp_, Ki_, Kd_;
    
    // PID limits
    double max_integral_, max_output_;
    
    // System state
    double target_yaw_, current_yaw_;
    double integral_, prev_error_;
    std::chrono::steady_clock::time_point prev_time_, start_time_;
    
    // Status flags
    bool yaw_tgt_rxd_, pose_rxd_, path_done_;
    
    // Low-pass filter
    double derivative_filter_alpha_, filtered_derivative_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawPIDController>());
    rclcpp::shutdown();
    return 0;
}
