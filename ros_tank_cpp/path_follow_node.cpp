#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <limits>

class PathFollowNode : public rclcpp::Node
{
public:
    PathFollowNode() : Node("path_follow_node")
    {
        // Create subscriptions
        path_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/path", 10, std::bind(&PathFollowNode::path_callback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/estimated_pose", 10, std::bind(&PathFollowNode::pose_callback, this, std::placeholders::_1));

        // Create publishers
        yaw_tgt_pub_ = this->create_publisher<std_msgs::msg::Float32>("/yaw_tgt", 10);
        path_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/path_done", 10);

        // Initialize variables
        position_.resize(3, 0.0);
        orientation_.resize(4, 0.0);
        pose_rxd_ = false;
        lookahead_dist_ = 0.15;
        last_point_ = false;
        path_rxd_ = false;
        yaw_ = 0.0;

        // Create timer (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PathFollowNode::timer_callback, this));
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        position_[0] = msg->position.x;
        position_[1] = msg->position.y;
        position_[2] = msg->position.z;

        // Convert quaternion to euler angles
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        tf2::Matrix3x3 mat(quat);
        double roll, pitch;
        mat.getRPY(roll, pitch, yaw_);
        
        pose_rxd_ = true;
    }

    std::pair<std::vector<double>, bool> find_lookahead_point()
    {
        // Find the closest point on the path
        double last_distance = 10000.0;
        int closest_idx = 0;
        
        for (int i = 0; i < static_cast<int>(path_.size()); i++)
        {
            double dx = path_[i][0] - position_[0];
            double dy = path_[i][1] - position_[1];
            double dz = path_[i][2] - position_[2];
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance > last_distance)
            {
                break;
            }
            closest_idx = i;
            last_distance = distance;
        }
        
        if (closest_idx > 0)
        {
            // Remove points before closest_idx-1
            path_.erase(path_.begin(), path_.begin() + closest_idx - 1);
        }

        // Start searching from the closest point onward
        for (int i = 0; i < static_cast<int>(path_.size()); i++)
        {
            double dx = path_[i][0] - position_[0];
            double dy = path_[i][1] - position_[1];
            double dz = path_[i][2] - position_[2];
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (dist >= lookahead_dist_)
            {
                // Found the lookahead point
                return std::make_pair(path_[i], false);
            }
        }

        // If no point is found beyond lookahead distance, return the last point
        return std::make_pair(path_.back(), true);
    }

    void path_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // The path is expected to be a sequence of [x1,y1,z1, x2,y2,z2, ...]
        if (!path_rxd_)
        {
            path_.clear();
            
            // Reshape data into Nx3 array
            for (size_t i = 0; i < msg->data.size(); i += 3)
            {
                if (i + 2 < msg->data.size())
                {
                    std::vector<double> point = {
                        msg->data[i],
                        msg->data[i + 1],
                        msg->data[i + 2]
                    };
                    path_.push_back(point);
                }
            }
            
            path_rxd_ = true;
            // RCLCPP_INFO(this->get_logger(), "Received new path with %lu points", path_.size());
        }
    }

    double normalize_angle(double angle)
    {
        while (angle < -M_PI)
        {
            angle += 2 * M_PI;
        }
        while (angle > M_PI)
        {
            angle -= 2 * M_PI;
        }
        return angle;
    }

    double calculate_yaw_tgt(const std::vector<double>& lookahead_point)
    {
        if (lookahead_point.empty())
        {
            return 0.0;
        }

        double dx = lookahead_point[0] - position_[0];
        double dy = lookahead_point[1] - position_[1];

        double target_yaw = std::atan2(dy, dx);

        return target_yaw;
    }

    void path_follow()
    {
        if (!path_rxd_)
        {
            RCLCPP_INFO(this->get_logger(), "No Path!");
            return;
        }

        if (!pose_rxd_)
        {
            RCLCPP_INFO(this->get_logger(), "No Pose!");
            return;
        }

        if (path_done_.data)
        {
            path_done_pub_->publish(path_done_);
            return;
        }

        if (path_.empty())
        {
            return;
        }

        auto [lookahead_point, last_point] = find_lookahead_point();
        //RCLCPP_INFO(this->get_logger(), "Path done: %s", last_point ? "true" : "false");
        
        double target_yaw = calculate_yaw_tgt(lookahead_point);

        if (lookahead_point.empty())
        {
            return;
        }
        
        //RCLCPP_INFO(this->get_logger(), "Lookahead Point: %.3f, %.3f", 
        //            lookahead_point[0], lookahead_point[1]);

        // Normalize target yaw
        target_yaw = normalize_angle(target_yaw);

        path_done_.data = last_point;
        
        // Publish messages
        auto target_yaw_msg = std_msgs::msg::Float32();
        target_yaw_msg.data = target_yaw;
        yaw_tgt_pub_->publish(target_yaw_msg);
        path_done_pub_->publish(path_done_);
    }

    void timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "Following Path");
        path_follow();
    }

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_tgt_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_done_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    std::vector<double> position_;
    std::vector<double> orientation_;
    bool pose_rxd_;
    std::vector<std::vector<double>> path_;
    double lookahead_dist_;
    bool last_point_;
    bool path_rxd_;
    std_msgs::msg::Bool path_done_;
    double yaw_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowNode>());
    rclcpp::shutdown();
    return 0;
}
