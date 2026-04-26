
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>

class EncoderPublisher : public rclcpp::Node
{
public:
    EncoderPublisher() : Node("encoder_publisher")
    {
        // Create publishers
        right_publisher_ = this->create_publisher<std_msgs::msg::Int32>("encoder/right", 10);
        left_publisher_ = this->create_publisher<std_msgs::msg::Int32>("encoder/left", 10);

        // Initialize messages
        right_msg_ = std_msgs::msg::Int32();
        left_msg_ = std_msgs::msg::Int32();
        
        // Initialize the line buffer
        line_buffer_.clear();

        // Create timer (50ms)
        timer_period_ = std::chrono::milliseconds(50);
        timer_ = this->create_wall_timer(timer_period_,
                                        std::bind(&EncoderPublisher::timer_callback, this));

        // Initialize serial connection
        serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port /dev/ttyACM0");
            rclcpp::shutdown();
            return;
        }

        configure_serial_port();
        RCLCPP_INFO(this->get_logger(), "Serial connection established");
    }

    ~EncoderPublisher()
    {
        if (serial_fd_ != -1) {
            close(serial_fd_);
        }
    }

private:
    void configure_serial_port()
    {
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error getting serial attributes");
            close(serial_fd_);
            rclcpp::shutdown();
            return;
        }

        // Set baud rate
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        // Configure port settings
        tty.c_cflag &= ~PARENB;        // No parity
        tty.c_cflag &= ~CSTOPB;        // 1 stop bit
        tty.c_cflag &= ~CSIZE;         // Clear data size bits
        tty.c_cflag |= CS8;            // 8 data bits
        tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable reading, ignore modem controls

        // Configure input settings
        tty.c_lflag &= ~ICANON;        // Non-canonical mode
        tty.c_lflag &= ~ECHO;          // Disable echo
        tty.c_lflag &= ~ECHOE;         // Disable erasure
        tty.c_lflag &= ~ECHONL;        // Disable new-line echo
        tty.c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error setting serial attributes");
            close(serial_fd_);
            rclcpp::shutdown();
            return;
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }//    ~EncoderPublisher()
    //{
    //    if (serial_fd_ != -1) {
    //        close(serial_fd_);
    //    }
    //}


    void timer_callback()
    {
        read_serial();
        right_publisher_->publish(right_msg_);
        left_publisher_->publish(left_msg_);
    }

    void read_serial()
    {
        char buffer[256];  // Larger buffer
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            line_buffer_ += std::string(buffer);  // Accumulate all incoming data

            // Process ALL complete lines in the buffer
            size_t newline_pos;
            while ((newline_pos = line_buffer_.find('\n')) != std::string::npos) {
                // Extract one complete line
                std::string complete_line = line_buffer_.substr(0, newline_pos);
                
                // Remove the processed line from buffer (including the \n)
                line_buffer_.erase(0, newline_pos + 1);

                // Process this complete line
                process_line(complete_line);
            }

            // Safety: prevent buffer from growing too large if no newlines come
            if (line_buffer_.size() > 1000) {
                RCLCPP_WARN(this->get_logger(), "Line buffer overflow - clearing buffer");
                line_buffer_.clear();
            }
        }
        else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
        }
    }

    void process_line(const std::string& line)
    {
        // Remove any trailing whitespace
        std::string clean_line = line;
        clean_line.erase(clean_line.find_last_not_of(" \t\n\r\f\v") + 1);

        // Skip empty lines
        if (clean_line.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Line Empty");
            return;
        }

        // Debug: log the exact line we're processing
        RCLCPP_DEBUG(this->get_logger(), "Processing line: '%s'", clean_line.c_str());

        // Split the line by spaces
        std::vector<std::string> tokens = split_string(clean_line, ' ');

        if (tokens.size() >= 2) {
            try {
                int left_val = std::stoi(tokens[0]);
                int right_val = std::stoi(tokens[1]);
                
                // Only update if we successfully parsed both values
                left_msg_.data = left_val;
                right_msg_.data = right_val;
                
                // Optional: log every few successful reads
                //static int count = 0;
               // if (++count % 20 == 0) {
               //    RCLCPP_INFO(this->get_logger(), "Encoder values: L=%d, R=%d", left_val, right_val);
               // }
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse line: '%s' - %s", 
                           clean_line.c_str(), e.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Line has insufficient tokens: '%s' (tokens: %zu)", 
                       clean_line.c_str(), tokens.size());
        }
    }

    std::vector<std::string> split_string(const std::string& str, char delimiter)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;

        while (std::getline(ss, token, delimiter)) {
            if (!token.empty()) {
                tokens.push_back(token);
            }
        }

        return tokens;
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_publisher_;

    // Messages
    std_msgs::msg::Int32 right_msg_;
    std_msgs::msg::Int32 left_msg_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds timer_period_;

    // Serial connection
    int serial_fd_;
    
    // Line buffering
    std::string line_buffer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto encoder_publisher = std::make_shared<EncoderPublisher>();

    rclcpp::spin(encoder_publisher);

    rclcpp::shutdown();
    return 0;
}
