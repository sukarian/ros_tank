#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>
#include <sys/file.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

// MPU6050 Registers
const int IMU_ADDR = 0x68;      // I2C address of MPU6050
const int PWR_MGMT_1 = 0x6B;
const int ACCEL_XOUT_H = 0x3B;
const int GYRO_XOUT_H = 0x43;

// MPU6050 Configuration
const double ACCEL_SCALE = 16384.0;  // for ±2g range
const double GYRO_SCALE = 131.0;     // for ±250°/s range

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("imu_node"), i2c_fd_(-1), mutex_fd_(-1)
    {
        // Initialize publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // Open I2C device
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device %s", strerror(errno));
            return;
        }

        // Set I2C slave address
        if (ioctl(i2c_fd_, I2C_SLAVE, IMU_ADDR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set I2C slave address");
            close(i2c_fd_);
            return;
        }

        // Open mutex file
        mutex_fd_ = open("/tmp/gpio_mutex", O_WRONLY | O_CREAT, 0644);
        if (mutex_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open mutex file");
            close(i2c_fd_);
            return;
        }

        // Initialize MPU6050
        try {
            // Wake up MPU6050 by writing 0 to PWR_MGMT_1 register
            write_byte_data(PWR_MGMT_1, 0);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6050: %s", e.what());
            cleanup();
            return;
        }
        flock(mutex_fd_, LOCK_UN);

        // Create timer (100ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ImuNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "IMU Node initialized");
    }

    ~ImuNode()
    {
        cleanup();
    }

private:
    void write_byte_data(int reg, int data)
    {
        char buffer[2];
        buffer[0] = reg;
        buffer[1] = data;
        
        if (write(i2c_fd_, buffer, 2) != 2) {
            throw std::runtime_error("Failed to write to I2C device");
        }
    }

    int read_byte_data(int reg)
    {
        char reg_addr = reg;
        char data;
        
        // Write register address
        if (write(i2c_fd_, &reg_addr, 1) != 1) {
            throw std::runtime_error("Failed to write register address");
        }
        
        // Read data
        if (read(i2c_fd_, &data, 1) != 1) {
            throw std::runtime_error("Failed to read from I2C device");
        }
        
        return static_cast<unsigned char>(data);
    }

    int16_t read_raw_data(int addr)
    {
        flock(mutex_fd_, LOCK_EX);
        try {
            int high = read_byte_data(addr);
            int low = read_byte_data(addr + 1);
            int16_t data = (high << 8) | low;
            return data;
        } catch (const std::exception& e) {
            flock(mutex_fd_, LOCK_UN);
            throw;
        }
        flock(mutex_fd_, LOCK_UN);
    }

    void read_imu_data(double& accel_x, double& accel_y, double& accel_z,
                       double& gyro_x, double& gyro_y, double& gyro_z)
    {
        accel_x = read_raw_data(ACCEL_XOUT_H) / ACCEL_SCALE * -1.0;
        accel_y = read_raw_data(ACCEL_XOUT_H + 2) / ACCEL_SCALE;
        accel_z = read_raw_data(ACCEL_XOUT_H + 4) / ACCEL_SCALE;
        
        gyro_x = read_raw_data(GYRO_XOUT_H) / GYRO_SCALE;
        gyro_y = read_raw_data(GYRO_XOUT_H + 2) / GYRO_SCALE;
        gyro_z = read_raw_data(GYRO_XOUT_H + 4) / GYRO_SCALE;
    }

    void timer_callback()
    {
        try {
            double accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
            read_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

            auto imu_msg = sensor_msgs::msg::Imu();
            
            // Set header
            imu_msg.header.stamp = this->get_clock()->now();
            imu_msg.header.frame_id = "imu_link";
            
            // Set linear acceleration (convert to m/s²)
            imu_msg.linear_acceleration.x = accel_x * 9.81;
            imu_msg.linear_acceleration.y = accel_y * 9.81;
            imu_msg.linear_acceleration.z = accel_z * 9.81;
            
            // Set angular velocity (convert to rad/s)
            imu_msg.angular_velocity.x = gyro_x * M_PI / 180.0;
            imu_msg.angular_velocity.y = gyro_y * M_PI / 180.0;
            imu_msg.angular_velocity.z = gyro_z * M_PI / 180.0;
            
            // Publish the message
            imu_pub_->publish(imu_msg);
            
            // Uncomment for debugging
            // RCLCPP_INFO(this->get_logger(), "Angular velocity: x=%.3f, y=%.3f, z=%.3f", 
            //             imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading IMU data: %s", e.what());
        }
    }

    void cleanup()
    {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
            i2c_fd_ = -1;
        }
        if (mutex_fd_ >= 0) {
            close(mutex_fd_);
            mutex_fd_ = -1;
        }
    }

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // I2C and mutex file descriptors
    int i2c_fd_;
    int mutex_fd_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<ImuNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("imu_node"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
