#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("kinematics_node")
    {
        // Create subscription for cmd_vel
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&KinematicsNode::twist_callback, this, std::placeholders::_1));
        
        // Create publishers for motor commands
        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor_cmd/right", 10);
        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor_cmd/left", 10);
        
        // Initialize twist message
        twist_ = geometry_msgs::msg::Twist();
        
        // Declare and get parameter
        this->declare_parameter("track_base", 0.18);
        track_base_ = this->get_parameter("track_base").as_double();
        
        // Create timer (50Hz - 0.02 seconds)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&KinematicsNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Initialize speed messages
        auto left_speed = std_msgs::msg::Float32();
        auto right_speed = std_msgs::msg::Float32();
        
        // Reset speeds
        left_speed.data = 0.0;
        right_speed.data = 0.0;
        
        // Add linear velocity component
        left_speed.data += twist_.linear.x;
        right_speed.data += twist_.linear.x;
        
        // Add angular velocity component
        left_speed.data -= twist_.angular.z * track_base_ / 2.0;
        right_speed.data += twist_.angular.z * track_base_ / 2.0;
        
        // Publish motor commands
        right_wheel_pub_->publish(right_speed);
        left_wheel_pub_->publish(left_speed);
    }
    
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        twist_ = *msg;
    }
    
    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wheel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::Twist twist_;
    double track_base_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
