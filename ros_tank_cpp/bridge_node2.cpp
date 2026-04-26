#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sys/file.h>
#include <unistd.h>
#include <csignal>
#include <fcntl.h>
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>

class BridgeNode : public rclcpp::Node
{
public:
    BridgeNode() : Node("h_bridge_node")
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

        RCLCPP_INFO(this->get_logger(), "Starting H-Bridge Node initialization");

        // Check if PWM is already available
        if (check_pwm_available()) {
            RCLCPP_INFO(this->get_logger(), "PWM is already available");
            pwm_available_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "PWM not available, using software control");
            pwm_available_ = false;
        }

        // Initialize GPIO for motor direction control
        if (initialize_gpio()) {
            RCLCPP_INFO(this->get_logger(), "GPIO initialized successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "GPIO initialization failed");
            simulation_mode_ = true;
        }

        // Create timer (100ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BridgeNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "H-Bridge Node initialized %s", 
                   simulation_mode_ ? "(SIMULATION MODE)" : "(HARDWARE MODE)");
    }

    ~BridgeNode()
    {
        cleanup();
    }

private:
    bool check_pwm_available()
    {
        // Check if any PWM channels are already available
        std::vector<std::string> pwm_paths = {
            "/sys/class/pwm/pwmchip0/pwm0/",
            "/sys/class/pwm/pwmchip1/pwm0/",
            "/sys/class/pwm/pwmchip2/pwm0/",
            "/sys/class/pwm/pwmchip3/pwm0/"
        };

        for (const auto& path : pwm_paths) {
            if (std::filesystem::exists(path)) {
                // Check if we can write to duty_cycle
                std::ofstream test_file(path + "duty_cycle");
                if (test_file.is_open()) {
                    test_file << "0";
                    if (!test_file.fail()) {
                        pwm_path_ = path;
                        RCLCPP_INFO(this->get_logger(), "Found available PWM: %s", path.c_str());
                        return true;
                    }
                }
            }
        }

        // Try to export a PWM channel if none are available
        return try_export_pwm();
    }

    bool try_export_pwm()
    {
        std::vector<std::string> pwm_chips = {
            "/sys/class/pwm/pwmchip0/",
            "/sys/class/pwm/pwmchip1/",
            "/sys/class/pwm/pwmchip2/",
            "/sys/class/pwm/pwmchip3/"
        };

        for (const auto& chip : pwm_chips) {
            if (std::filesystem::exists(chip)) {
                try {
                    // Try to export channel 0
                    std::ofstream export_file(chip + "export");
                    if (!export_file.is_open()) {
                        continue;
                    }
                    export_file << "0";
                    export_file.close();

                    if (export_file.fail()) {
                        RCLCPP_DEBUG(this->get_logger(), "Failed to export PWM on %s", chip.c_str());
                        continue;
                    }

                    // Wait for PWM channel to be created
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    std::string pwm_path = chip + "pwm0/";
                    if (std::filesystem::exists(pwm_path)) {
                        // Configure PWM
                        if (configure_pwm(pwm_path)) {
                            pwm_path_ = pwm_path;
                            RCLCPP_INFO(this->get_logger(), "Successfully exported and configured PWM: %s", pwm_path.c_str());
                            return true;
                        }
                    }

                } catch (...) {
                    // Continue to next chip
                }
            }
        }

        return false;
    }

    bool configure_pwm(const std::string& pwm_path)
    {
        try {
            // Set period (1ms = 1000000ns for 1kHz)
            std::ofstream period_file(pwm_path + "period");
            if (!period_file.is_open()) {
                return false;
            }
            period_file << "1000000";
            period_file.close();

            if (period_file.fail()) {
                return false;
            }

            // Set duty cycle to 0
            std::ofstream duty_file(pwm_path + "duty_cycle");
            if (!duty_file.is_open()) {
                return false;
            }
            duty_file << "0";
            duty_file.close();

            if (duty_file.fail()) {
                return false;
            }

            // Enable PWM
            std::ofstream enable_file(pwm_path + "enable");
            if (!enable_file.is_open()) {
                return false;
            }
            enable_file << "1";
            enable_file.close();

            return !enable_file.fail();

        } catch (...) {
            return false;
        }
    }

    bool set_pwm_duty_cycle(double duty_cycle)
    {
        if (simulation_mode_ || !pwm_available_ || pwm_path_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "SIMULATION: PWM duty cycle = %.1f%%", duty_cycle * 100);
            return true;
        }

        try {
            // Convert duty cycle (0.0 to 1.0) to nanoseconds (0 to 1000000)
            int duty_ns = static_cast<int>(duty_cycle * 1000000);
            duty_ns = std::clamp(duty_ns, 0, 1000000);
            
            std::ofstream duty_file(pwm_path_ + "duty_cycle");
            if (!duty_file.is_open()) {
                RCLCPP_WARN(this->get_logger(), "Failed to open PWM duty cycle file");
                return false;
            }
            
            duty_file << duty_ns;
            duty_file.close();
            
            if (duty_file.fail()) {
                RCLCPP_WARN(this->get_logger(), "Failed to write PWM duty cycle");
                return false;
            }
            
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "PWM set error: %s", e.what());
            return false;
        }
    }

    bool initialize_gpio()
    {
        try {
            RCLCPP_INFO(this->get_logger(), "Initializing GPIO for motor direction...");

            // Try common physical pin configurations
            std::vector<int> physical_pins = {11, 12, 13, 15};  // Common physical pins
            
            for (size_t i = 0; i < physical_pins.size(); i++) {
                int gpio_number = 512 + (physical_pins[i] - 1);  // Convert to GPIO number
                
                if (!export_gpio(gpio_number)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to export GPIO %d", gpio_number);
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                
                if (!set_gpio_direction(gpio_number, "out")) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set direction for GPIO %d", gpio_number);
                    return false;
                }

                if (!set_gpio_value(gpio_number, 0)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPIO %d", gpio_number);
                    return false;
                }

                // Store the GPIO numbers
                if (i == 0) in1_ = gpio_number;
                if (i == 1) in2_ = gpio_number;
                if (i == 2) in3_ = gpio_number;
                if (i == 3) in4_ = gpio_number;

                RCLCPP_INFO(this->get_logger(), "Initialized GPIO %d (physical pin %d)", 
                           gpio_number, physical_pins[i]);
            }

            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "GPIO initialization error: %s", e.what());
            return false;
        }
    }

    bool export_gpio(int gpio_number)
    {
        std::string gpio_path = "/sys/class/gpio/gpio" + std::to_string(gpio_number);
        
        if (std::filesystem::exists(gpio_path)) {
            RCLCPP_INFO(this->get_logger(), "GPIO %d already exported", gpio_number);
            return true;
        }

        std::ofstream export_file("/sys/class/gpio/export");
        if (!export_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open export file");
            return false;
        }
        
        export_file << gpio_number;
        export_file.close();
        
        if (export_file.fail()) {
            RCLCPP_ERROR(this->get_logger(), "Export failed for GPIO %d", gpio_number);
            return false;
        }

        // Wait for GPIO to be available
        int attempts = 0;
        while (!std::filesystem::exists(gpio_path) && attempts < 20) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            attempts++;
        }

        return std::filesystem::exists(gpio_path);
    }

    bool set_gpio_direction(int gpio_number, const std::string& direction)
    {
        std::string path = "/sys/class/gpio/gpio" + std::to_string(gpio_number) + "/direction";
        
        int attempts = 0;
        while (!std::filesystem::exists(path) && attempts < 20) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            attempts++;
        }

        if (!std::filesystem::exists(path)) {
            return false;
        }

        std::ofstream direction_file(path);
        if (!direction_file.is_open()) {
            return false;
        }
        
        direction_file << direction;
        direction_file.close();
        
        return !direction_file.fail();
    }

    bool set_gpio_value(int gpio_number, int value)
    {
        if (simulation_mode_) {
            RCLCPP_DEBUG(this->get_logger(), "SIMULATION: GPIO %d = %d", gpio_number, value);
            return true;
        }

        std::string path = "/sys/class/gpio/gpio" + std::to_string(gpio_number) + "/value";
        std::ofstream value_file(path);
        
        if (!value_file.is_open()) {
            return false;
        }
        
        value_file << value;
        value_file.close();
        
        return !value_file.fail();
    }

    void unexport_gpio(int gpio_number)
    {
        if (simulation_mode_) return;

        std::string gpio_path = "/sys/class/gpio/gpio" + std::to_string(gpio_number);
        if (!std::filesystem::exists(gpio_path)) {
            return;
        }

        std::ofstream unexport_file("/sys/class/gpio/unexport");
        if (unexport_file.is_open()) {
            unexport_file << gpio_number;
            unexport_file.close();
        }
    }

    void right_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received right serial: %d", msg->data);
        right_speed_ = *msg;
    }

    void left_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received left serial: %d", msg->data);
        left_speed_ = *msg;
    }

    void timer_callback()
    {
        move_motors();
    }

    void move_motors()
    {
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

            // Calculate PWM duty cycles (0.0 to 1.0)
            double duty_right = std::abs(right_speed_.data) / 250.0;
            double duty_left = std::abs(left_speed_.data) / 250.0;

            // Apply PWM to motors (use the higher duty cycle)
            double max_duty = std::max(duty_right, duty_left);
            if (!set_pwm_duty_cycle(max_duty)) {
                RCLCPP_WARN(this->get_logger(), "Failed to set PWM duty cycle");
            }

            RCLCPP_DEBUG(this->get_logger(), "Motors - Left: %d (%.1f%%), Right: %d (%.1f%%)", 
                        left_speed_.data, duty_left * 100, right_speed_.data, duty_right * 100);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in move_motors: %s", e.what());
        }
    }

    void left_stop()
    {
        if (set_gpio_value(in3_, 0) && set_gpio_value(in4_, 0)) {
            RCLCPP_DEBUG(this->get_logger(), "Left motor stopped");
        }
    }

    void left_forward()
    {
        if (set_gpio_value(in3_, 1) && set_gpio_value(in4_, 0)) {
            RCLCPP_DEBUG(this->get_logger(), "Left motor forward");
        }
    }

    void left_backward()
    {
        if (set_gpio_value(in3_, 0) && set_gpio_value(in4_, 1)) {
            RCLCPP_DEBUG(this->get_logger(), "Left motor backward");
        }
    }

    void right_stop()
    {
        if (set_gpio_value(in1_, 0) && set_gpio_value(in2_, 0)) {
            RCLCPP_DEBUG(this->get_logger(), "Right motor stopped");
        }
    }

    void right_forward()
    {
        if (set_gpio_value(in1_, 1) && set_gpio_value(in2_, 0)) {
            RCLCPP_DEBUG(this->get_logger(), "Right motor forward");
        }
    }

    void right_backward()
    {
        if (set_gpio_value(in1_, 0) && set_gpio_value(in2_, 1)) {
            RCLCPP_DEBUG(this->get_logger(), "Right motor backward");
        }
    }

    void cleanup()
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up...");
        
        // Cleanup GPIO
        if (!simulation_mode_) {
            std::vector<int> gpio_pins = {in1_, in2_, in3_, in4_};
            for (int pin : gpio_pins) {
                if (pin != -1) {
                    unexport_gpio(pin);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Cleanup completed");
    }

    // Member variables
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_serial_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_serial_cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Int32 right_speed_;
    std_msgs::msg::Int32 left_speed_;
    int in1_ = -1, in2_ = -1, in3_ = -1, in4_ = -1;
    std::string pwm_path_;
    bool simulation_mode_ = false;
    bool pwm_available_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting H-Bridge Node");
    
    try {
        auto node = std::make_shared<BridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
