#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sys/file.h>
#include <unistd.h>
#include <lgpio.h>
#include <csignal>

class BridgeNode : public rclcpp::Node
{
public:

    
    
    BridgeNode() : Node("h_bridge_node"), chip_(-1), mutex_fd_(-1)
    {
        // Initialize subscribers
        right_serial_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/serial/motor_cmd/right", 10,
            std::bind(&BridgeNode::right_cmd_callback, this, std::placeholders::_1));

        left_serial_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/serial/motor_cmd/left", 10,
            std::bind(&BridgeNode::left_cmd_callback, this, std::placeholders::_1));

        // Initialize message objects
        right_speed_.data = 0;
        left_speed_.data = 0;
        
        right_cycles_passed_ = 0;
        left_cycles_passed_ = 0;
        
        right_cycles_total_ = 0;
        left_cycles_total_ = 0;
        
        // Declare parameters
        this->declare_parameter("PWM_CYCLES", 100);
        
        // Get parameter values
        pwm_cycles_ = this->get_parameter("PWM_CYCLES").as_int();

        // GPIO pin definitions
        enA_ = 18;
        in1_ = 23;
        in2_ = 24;
        in3_ = 25;
        in4_ = 12;
        enB_ = 13;


        try {
            chip_ = lgGpiochipOpen(4);
            if (chip_ < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip");
                return;
            }

            lgGpioClaimOutput(chip_, 0, enA_, 0);
            lgGpioClaimOutput(chip_, 0, in1_, 0);
            lgGpioClaimOutput(chip_, 0, in2_, 0);
            lgGpioClaimOutput(chip_, 0, in3_, 0);
            lgGpioClaimOutput(chip_, 0, in4_, 0);
            lgGpioClaimOutput(chip_, 0, enB_, 0);

            RCLCPP_INFO(this->get_logger(), "GPIO initialized successfully");
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO");
        }

        // Timer configuration
        timer_period_ = 1 / pwm_cycles_; // seconds

        // Create timer (100ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&BridgeNode::timer_callback, this));

        // Sleep for 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "H-Bridge Node initialized");
    }

    ~BridgeNode()
    {
        cleanup();
    }
    
    enum class Phase{
      INIT,
      ON,
      OFF
    };
private:

    Phase right_phase_ = Phase::INIT;
    Phase left_phase_ = Phase::INIT;
    
    void right_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received right serial: %d", msg->data);
        right_speed_ = *msg;
    }

    void left_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received left serial: %d", msg->data);
        left_speed_ = *msg;
    }

    void timer_callback()
    {
        move_motors();
    }

    void left_on()
    {
      if (left_speed_.data > 0) {
          left_forward();
      } else if (left_speed_.data < 0) {
          left_backward();
      } 
      left_cycles_passed_++;
      if (left_cycles_passed_ > left_cycles_total_){
        left_phase_ = Phase::OFF;
      }
    }
    
    void left_off()
    {
      left_stop();
      left_cycles_passed_++;
      if (left_cycles_passed_ > pwm_cycles_){
        left_phase_ = Phase::INIT;
      }
    }
    
    void right_on()
    {
      if (right_speed_.data > 0) {
          right_forward();
      } else if (right_speed_.data < 0) {
          right_backward();
      } 
      right_cycles_passed_++;
      if (right_cycles_passed_ > right_cycles_total_){
        right_phase_ = Phase::OFF;
      }
    }
    
    void right_off()
    {
      right_stop();
      right_cycles_passed_++;
      if (right_cycles_passed_ > pwm_cycles_){
        right_phase_ = Phase::INIT;
      }
    }
    
    void move_motors()
    {
      switch(left_phase_){
        case Phase::INIT:
          left_cycles_total_ = std::abs(left_speed_.data);
          left_cycles_passed_ = 0;
          //RCLCPP_INFO(this->get_logger(), "Left Cycles: %d", left_cycles_total_);
          left_phase_ = Phase::ON;
          break;
        case Phase::ON:
          left_on();
          //RCLCPP_INFO(this->get_logger(), "Left ON");
          //RCLCPP_INFO(this->get_logger(), "Left Cycles Passed: %d", left_cycles_passed_);
          break;
        case Phase::OFF:
        //RCLCPP_INFO(this->get_logger(), "Left OFF");
          left_off();
          break;
      }
          
      switch(right_phase_){
        case Phase::INIT:
          right_cycles_total_ = std::abs(right_speed_.data);
          right_cycles_passed_ = 0;
          right_phase_ = Phase::ON;
          break;
        case Phase::ON:
          right_on();
          break;
        case Phase::OFF:
          right_off();
          break;
      }
    }

    void left_stop()
    {
        try {
            lgGpioWrite(chip_, in3_, 0);
            lgGpioWrite(chip_, in4_, 0);
            lgGpioWrite(chip_, enB_, 0);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in left_stop");
        }
    }

    void left_forward()
    {
        try {
            lgGpioWrite(chip_, in3_, 1);
            lgGpioWrite(chip_, in4_, 0);
            lgGpioWrite(chip_, enB_, 1);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in left_forward");
        }
    }

    void left_backward()
    {
        try {
            lgGpioWrite(chip_, in3_, 0);
            lgGpioWrite(chip_, in4_, 1);
            lgGpioWrite(chip_, enB_, 1);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in left_backward");
        }
    }

    void right_stop()
    {
        try {
            lgGpioWrite(chip_, in1_, 0);
            lgGpioWrite(chip_, in2_, 0);
            lgGpioWrite(chip_, enA_, 0);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in right_stop");
        }
    }

    void right_forward()
    {
        try {
            lgGpioWrite(chip_, in1_, 1);
            lgGpioWrite(chip_, in2_, 0);
            lgGpioWrite(chip_, enA_, 1);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in right_forward");
        }
    }

    void right_backward()
    {
        try {
            lgGpioWrite(chip_, in1_, 0);
            lgGpioWrite(chip_, in2_, 1);
            lgGpioWrite(chip_, enA_, 1);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in right_backward");
        }
    }
    
    void cleanup()
    {
        try {
            if (chip_ >= 0) {
                left_stop();
                right_stop();
                lgGpiochipClose(chip_);
            }
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error during cleanup");
        }
    }

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_serial_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_serial_cmd_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Message objects
    std_msgs::msg::Int32 right_speed_;
    std_msgs::msg::Int32 left_speed_;

    // GPIO configuration
    int chip_;
    int mutex_fd_;
    int enA_, in1_, in2_, in3_, in4_, enB_;
    int left_cycles_passed_, right_cycles_passed_;
    int left_cycles_total_, right_cycles_total_;
    int pwm_cycles_;

    // Timing configuration
    double timer_period_;
    double duty_period_;
};

// Global pointer for signal handler
std::shared_ptr<BridgeNode> global_node_ptr;

void signal_handler(int signum)
{
    RCLCPP_INFO(rclcpp::get_logger("bridge_node"), "Interrupt signal (%d) received. Shutting down...", signum);
    if (global_node_ptr) {
        // The destructor will handle cleanup
        global_node_ptr.reset();
    }
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Set up signal handler
    std::signal(SIGINT, signal_handler);

    try {
        global_node_ptr = std::make_shared<BridgeNode>();
        rclcpp::spin(global_node_ptr);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("bridge_node"), "Exception: %s", e.what());
    }

    // Cleanup
    if (global_node_ptr) {
        global_node_ptr.reset();
    }
    rclcpp::shutdown();
    return 0;
}
