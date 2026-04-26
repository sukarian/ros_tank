#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode() : Node("motor_control_node")
    {
        // Initialize subscribers
        path_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/path_done", 10, 
            std::bind(&MotorControlNode::path_done_callback, this, std::placeholders::_1));
        
        right_wheel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wheel_speed/right", 10,
            std::bind(&MotorControlNode::right_feedback_callback, this, std::placeholders::_1));
        
        left_wheel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wheel_speed/left", 10,
            std::bind(&MotorControlNode::left_feedback_callback, this, std::placeholders::_1));
        
        right_motor_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor_cmd/right", 10,
            std::bind(&MotorControlNode::right_cmd_callback, this, std::placeholders::_1));
        
        left_motor_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor_cmd/left", 10,
            std::bind(&MotorControlNode::left_cmd_callback, this, std::placeholders::_1));

        // Initialize publishers
        right_serial_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("/serial/motor_cmd/right", 10);
        left_serial_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("/serial/motor_cmd/left", 10);

        // Declare parameters
        this->declare_parameter("Kp", 1);
        this->declare_parameter("Ki", 1);
        this->declare_parameter("Kd", 1);
        this->declare_parameter("deadband", 1);
        this->declare_parameter("PWM_CYCLES", 100);

        // Get parameter values
        left_Kp_ = this->get_parameter("Kp").as_int();
        left_Ki_ = this->get_parameter("Ki").as_int();
        left_Kd_ = this->get_parameter("Kd").as_int();
        right_Kp_ = this->get_parameter("Kp").as_int();
        right_Ki_ = this->get_parameter("Ki").as_int();
        right_Kd_ = this->get_parameter("Kd").as_int();
        deadband_ = this->get_parameter("deadband").as_int();
        speed_limit_ = this->get_parameter("PWM_CYCLES").as_int();

        RCLCPP_INFO(this->get_logger(), "Kp: %d", right_Kp_);

        // Initialize variables
        left_error_int_ = 0.0;
        left_error_last_ = 0.0;
        right_error_int_ = 0.0;
        right_error_last_ = 0.0;
        path_done_ = false;
        
        right_motor_cmd_.data = 0.0;
        left_motor_cmd_.data = 0.0;
        right_wheel_.data = 0.0;
        left_wheel_.data = 0.0;

        // Initialize time points
        left_time_ = std::chrono::steady_clock::now();
        right_time_ = std::chrono::steady_clock::now();

        // Create timer (10ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorControlNode::timer_callback, this));

        // Sleep for 1 second (equivalent to time.sleep(1) in Python)
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

private:
    void path_done_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        path_done_ = msg->data;
    }

    void right_feedback_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        right_wheel_ = *msg;
    }

    void left_feedback_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        left_wheel_ = *msg;
    }

    void right_cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        right_motor_cmd_ = *msg;
    }

    void left_cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        left_motor_cmd_ = *msg;
    }

    void timer_callback()
    {
        if (path_done_) {
            right_serial_cmd_.data = 0;
            left_serial_cmd_.data = 0;
            right_serial_cmd_pub_->publish(right_serial_cmd_);
            left_serial_cmd_pub_->publish(left_serial_cmd_);
        } else {
            right_PID();
            left_PID();
        }
    }

    void right_PID()
    {
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - right_time_);
        double right_dt = duration.count() / 1000000.0; // Convert to seconds
        right_time_ = current_time;

        double speed_error = right_motor_cmd_.data - right_wheel_.data;
        right_error_last_ = speed_error;
        right_error_int_ += speed_error * right_dt;
        double d_error = (speed_error - right_error_last_) / right_dt;
        
        int cmd = static_cast<int>(right_Kp_ * speed_error + right_Ki_ * right_error_int_ + right_Kd_ * d_error);
        right_serial_cmd_.data = constrain(cmd, -1*speed_limit_, speed_limit_);
        right_serial_cmd_pub_->publish(right_serial_cmd_);
    }

    void left_PID()
    {
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - left_time_);
        double left_dt = duration.count() / 1000000.0; // Convert to seconds
        left_time_ = current_time;

        double speed_error = left_motor_cmd_.data - left_wheel_.data;
        left_error_last_ = speed_error;
        left_error_int_ += speed_error * left_dt;
        double d_error = (speed_error - left_error_last_) / left_dt;
        
        int cmd = static_cast<int>(left_Kp_ * speed_error + left_Ki_ * left_error_int_ + left_Kd_ * d_error);
        left_serial_cmd_.data = constrain(cmd, -1*speed_limit_, speed_limit_);
        left_serial_cmd_pub_->publish(left_serial_cmd_);
    }

    int constrain(int val, int min_val, int max_val)
    {
        int out_val = std::min(max_val, std::max(min_val, val));
        if (std::abs(out_val) < deadband_) {
            out_val = 0;
        }
        return out_val;
    }

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_done_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_motor_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_motor_cmd_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_serial_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_serial_cmd_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Message objects
    std_msgs::msg::Float32 right_motor_cmd_;
    std_msgs::msg::Float32 left_motor_cmd_;
    std_msgs::msg::Float32 right_wheel_;
    std_msgs::msg::Float32 left_wheel_;
    std_msgs::msg::Int32 right_serial_cmd_;
    std_msgs::msg::Int32 left_serial_cmd_;

    // PID parameters
    int left_Kp_, left_Ki_, left_Kd_;
    int right_Kp_, right_Ki_, right_Kd_;
    int deadband_;

    // PID variables
    double left_error_int_, left_error_last_;
    double right_error_int_, right_error_last_;
    
    // Time tracking
    std::chrono::steady_clock::time_point left_time_;
    std::chrono::steady_clock::time_point right_time_;

    // State variables
    bool path_done_;
    int speed_limit_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
