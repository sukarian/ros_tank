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

        // GPIO pin definitions
        enA_ = 18;
        in1_ = 23;
        in2_ = 24;
        in3_ = 25;
        in4_ = 12;
        enB_ = 13;

        // Open mutex file
        mutex_fd_ = open("/tmp/gpio_mutex", O_WRONLY | O_CREAT, 0644);
        if (mutex_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open mutex file");
            return;
        }

        // Initialize GPIO with non-blocking lock
        if (acquire_gpio_lock()) {
            try {
                chip_ = lgGpiochipOpen(4);
                if (chip_ < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip");
                    release_gpio_lock();
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
            release_gpio_lock();
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not acquire GPIO lock during initialization");
        }

        // Timer configuration
        timer_period_ = 0.1; // seconds
        duty_period_ = timer_period_;

        // Create timer (100ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&BridgeNode::timer_callback, this));

        // Sleep for 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "H-Bridge Node initialized");
    }

    ~BridgeNode()
    {
        cleanup();
    }

private:
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

    bool acquire_gpio_lock()
    {
        if (mutex_fd_ < 0) return false;
        return flock(mutex_fd_, LOCK_EX | LOCK_NB) == 0;
    }

    void release_gpio_lock()
    {
        if (mutex_fd_ >= 0) {
            flock(mutex_fd_, LOCK_UN);
        }
    }

    void move_motors()
    {
        // Try to acquire GPIO lock non-blocking
        if (!acquire_gpio_lock()) {
            RCLCPP_DEBUG(this->get_logger(), "GPIO busy, skipping motor command");
            return;
        }

        try {
            // Control left motor direction
            if (left_speed_.data > 0) {
                left_forward();
            } else if (left_speed_.data < 0) {
                left_backward();
            } else {
                left_stop();
            }

            // Control right motor direction
            if (right_speed_.data > 0) {
                right_forward();
            } else if (right_speed_.data < 0) {
                right_backward();
            } else {
                right_stop();
            }

            // Calculate duty cycles
            double duty_cycle_right = std::abs(right_speed_.data) / 250.0 * (timer_period_);
            double duty_cycle_left = std::abs(left_speed_.data) / 250.0 * (timer_period_);

            // Apply PWM with the higher duty cycle first
            if (duty_cycle_right > duty_cycle_left) {
                pwm(enA_, enB_, duty_cycle_right, duty_cycle_left);
            } else {
                pwm(enB_, enA_, duty_cycle_left, duty_cycle_right);
            }
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in move_motors");
        }

        release_gpio_lock();
    }

    void left_stop()
    {
        try {
            lgGpioWrite(chip_, in3_, 0);
            lgGpioWrite(chip_, in4_, 0);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in left_stop");
        }
    }

    void left_forward()
    {
        try {
            lgGpioWrite(chip_, in3_, 1);
            lgGpioWrite(chip_, in4_, 0);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in left_forward");
        }
    }

    void left_backward()
    {
        try {
            lgGpioWrite(chip_, in3_, 0);
            lgGpioWrite(chip_, in4_, 1);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in left_backward");
        }
    }

    void right_stop()
    {
        try {
            lgGpioWrite(chip_, in1_, 0);
            lgGpioWrite(chip_, in2_, 0);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in right_stop");
        }
    }

    void right_forward()
    {
        try {
            lgGpioWrite(chip_, in1_, 1);
            lgGpioWrite(chip_, in2_, 0);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in right_forward");
        }
    }

    void right_backward()
    {
        try {
            lgGpioWrite(chip_, in1_, 0);
            lgGpioWrite(chip_, in2_, 1);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in right_backward");
        }
    }

    void pwm(int en1, int en2, double duty_cycle1, double duty_cycle2)
    {
        double duty_cycle_diff = duty_cycle1 - duty_cycle2;

        try {
            lgGpioWrite(chip_, en1, 1);
            lgGpioWrite(chip_, en2, 1);

            // Sleep for the shorter duty cycle
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(duty_cycle2 * 1000000)));

            lgGpioWrite(chip_, en2, 0);

            // Sleep for the difference
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(duty_cycle_diff * 1000000)));

            lgGpioWrite(chip_, en1, 0);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Error in PWM");
        }

        // Sleep for the remaining period
        double remaining_time = duty_period_ - duty_cycle1;
        if (remaining_time > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(remaining_time * 1000000)));
        }
    }

    void cleanup()
    {
        if (acquire_gpio_lock()) {
            try {
                if (chip_ >= 0) {
                    left_stop();
                    right_stop();
                    lgGpiochipClose(chip_);
                }
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Error during cleanup");
            }
            release_gpio_lock();
        }

        if (mutex_fd_ >= 0) {
            close(mutex_fd_);
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
