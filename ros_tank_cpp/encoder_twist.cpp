#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cmath>

class TwistNode : public rclcpp::Node
{
public:
    TwistNode() : Node("twist_node")
    {
        // Initialize subscribers
        left_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/encoder/left", 10, 
            std::bind(&TwistNode::left_encoder_callback, this, std::placeholders::_1));
        
        right_encoder_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/encoder/right", 10,
            std::bind(&TwistNode::right_encoder_callback, this, std::placeholders::_1));

        // Initialize publishers
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/twist/encoder", 10);
        right_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/wheel_speed/right", 10);
        left_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/wheel_speed/left", 10);

        // Initialize variables
        last_right_encoder_ = 0;
        last_left_encoder_ = 0;
        first_cycle_left_ = true;
        first_cycle_right_ = true;
        left_time_ = std::chrono::high_resolution_clock::now();
        right_time_ = std::chrono::high_resolution_clock::now();

        // Create timer (20ms = 500Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TwistNode::timer_callback, this));
    }

private:
    void left_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        left_encoder_ = *msg;
	//RCLCPP_INFO(this->get_logger(), "Received Left");
        if (first_cycle_left_) {
            last_left_encoder_ = msg->data;
            first_cycle_left_ = false;
        }
    }

    void right_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        right_encoder_ = *msg;
        if (first_cycle_right_) {
            last_right_encoder_ = msg->data;
            first_cycle_right_ = false;
        }
    }

    void timer_callback()
    {
        left_process();
        right_process();
        compute_twist();
        
        twist_pub_->publish(twist_);
        right_speed_pub_->publish(right_speed_);
        left_speed_pub_->publish(left_speed_);
    }

    void left_process()
    {
        double left_encoder_m = convert_data(left_encoder_.data);
        auto current_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - left_time_);
        double left_dt = duration.count() / 1000000.0; // Convert to seconds
        left_time_ = current_time;
        
        double last_left_encoder_m = convert_data(last_left_encoder_);
        
        if ((last_left_encoder_ > 30000 && left_encoder_.data < -30000) || 
            (last_left_encoder_ < -30000 && left_encoder_.data > 30000)) {
            left_speed_.data = (last_left_encoder_m + left_encoder_m) / left_dt;
        } else {
            left_speed_.data = (left_encoder_m - last_left_encoder_m) / left_dt;
        }
        
        last_left_encoder_ = left_encoder_.data;
    }

    void right_process()
    {
        double right_encoder_m = convert_data(right_encoder_.data);
        auto current_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - right_time_);
        double right_dt = duration.count() / 1000000.0; // Convert to seconds
        right_time_ = current_time;
        //RCLCPP_INFO(this->get_logger(), "Right wheel %d", right_encoder_.data);
        
        double last_right_encoder_m = convert_data(last_right_encoder_);
        
        if ((last_right_encoder_ > 30000 && right_encoder_.data < -30000) || 
            (last_right_encoder_ < -30000 && right_encoder_.data > 30000)) {
            right_speed_.data = (last_right_encoder_m + right_encoder_m) / right_dt;
        } else {
            right_speed_.data = (right_encoder_m - last_right_encoder_m) / right_dt;
        }
        //RCLCPP_INFO(this->get_logger(), "Right wheel %4f", right_speed_.data);
        last_right_encoder_ = right_encoder_.data;
    }

    double convert_data(int32_t msg)
    {
        return msg / 600.0 * 0.05751 * M_PI;
    }

    void compute_twist()
    {
        twist_.linear.x = (right_speed_.data + left_speed_.data) / 2.0;
        twist_.angular.z = (right_speed_.data - left_speed_.data) / 0.155;
    }

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_encoder_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_speed_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Message objects
    geometry_msgs::msg::Twist twist_;
    std_msgs::msg::Int32 right_encoder_;
    std_msgs::msg::Int32 left_encoder_;
    std_msgs::msg::Float32 right_speed_;
    std_msgs::msg::Float32 left_speed_;
    
    // Time tracking
    std::chrono::high_resolution_clock::time_point left_time_;
    std::chrono::high_resolution_clock::time_point right_time_;
    
    // Encoder tracking
    int32_t last_right_encoder_;
    int32_t last_left_encoder_;
    bool first_cycle_left_;
    bool first_cycle_right_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistNode>());
    rclcpp::shutdown();
    return 0;
}
