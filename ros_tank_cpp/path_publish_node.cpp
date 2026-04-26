#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <filesystem>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        // Declare parameters
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<double>("interpolation_distance", 0.01);

        std::string config_file = this->get_parameter("config_file").as_string();
        interpolation_distance_ = this->get_parameter("interpolation_distance").as_double();

        RCLCPP_INFO(this->get_logger(), "Config file path: %s", config_file.c_str());
        RCLCPP_INFO(this->get_logger(), "Interpolation distance: %.3f meters", interpolation_distance_);

        // Create publishers
        raw_path_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("raw_path", 10);
        interpolated_path_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/path", 10);

        // Load waypoints from file
        if (config_file.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No config file specified. Use --ros-args -p config_file:=<path>");
            waypoints_.clear();
        }
        else
        {
            waypoints_ = load_waypoints_from_file(config_file);
        }

        if (!waypoints_.empty())
        {
            // Publish both raw and interpolated paths immediately
            publish_paths();

            // Set up a timer to periodically publish the paths
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1), std::bind(&PathPublisher::publish_paths, this));
        }
    }

private:
    std::vector<std::vector<double>> load_waypoints_from_file(const std::string& config_file)
    {
        std::vector<std::vector<double>> waypoints;
        
        try
        {
            RCLCPP_INFO(this->get_logger(), "Loading waypoints from %s", config_file.c_str());

            if (!std::filesystem::exists(config_file))
            {
                RCLCPP_ERROR(this->get_logger(), "Config file does not exist: %s", config_file.c_str());
                return waypoints;
            }

            YAML::Node data = YAML::LoadFile(config_file);

            if (!data || !data["path_data"])
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid YAML: missing path_data section");
                return waypoints;
            }

            YAML::Node path_data = data["path_data"];

            if (!path_data["waypoints"])
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid YAML: missing waypoints section");
                return waypoints;
            }

            YAML::Node waypoints_data = path_data["waypoints"];

            if (waypoints_data.IsSequence())
            {
                for (const auto& wp : waypoints_data)
                {
                    if (wp.IsSequence() && wp.size() >= 3)
                    {
                        // Format: [[x, y, z], [x, y, z], ...]
                        std::vector<double> point = {
                            wp[0].as<double>(),
                            wp[1].as<double>(),
                            wp[2].as<double>()
                        };
                        waypoints.push_back(point);
                    }
                    else if (wp.IsMap() && wp["x"] && wp["y"] && wp["z"])
                    {
                        // Format: [{x: val, y: val, z: val}, ...]
                        std::vector<double> point = {
                            wp["x"].as<double>(),
                            wp["y"].as<double>(),
                            wp["z"].as<double>()
                        };
                        waypoints.push_back(point);
                    }
                }
            }

            RCLCPP_INFO(this->get_logger(), "Loaded %lu waypoints", waypoints.size());
            return waypoints;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error loading waypoints: %s", e.what());
            return waypoints;
        }
    }

    std::vector<std::vector<double>> interpolate_path(const std::vector<std::vector<double>>& points)
    {
        if (points.empty())
        {
            return {{0.0, 0.0, 0.0}};  // Return origin if no points
        }

        // Prepend origin point (0,0,0)
        std::vector<std::vector<double>> points_with_origin;
        points_with_origin.push_back({0.0, 0.0, 0.0});
        
        for (const auto& point : points)
        {
            points_with_origin.push_back(point);
        }

        std::vector<std::vector<double>> interpolated;
        interpolated.push_back(points_with_origin[0]);  // Start with origin

        for (size_t i = 0; i < points_with_origin.size() - 1; i++)
        {
            const auto& start = points_with_origin[i];
            const auto& end = points_with_origin[i + 1];

            // Calculate distance between current segment points
            double dx = end[0] - start[0];
            double dy = end[1] - start[1];
            double dz = end[2] - start[2];
            double segment_length = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (segment_length == 0)
            {
                continue;  // Skip zero-length segments
            }

            // Calculate number of interpolation points needed
            int num_points = static_cast<int>(segment_length / interpolation_distance_);

            if (num_points > 0)
            {
                // Create linearly spaced points along the segment
                for (int j = 1; j <= num_points; j++)
                {
                    double t = static_cast<double>(j) / (num_points + 1);
                    double x = start[0] + t * dx;
                    double y = start[1] + t * dy;
                    double z = start[2] + t * dz;
                    
                    interpolated.push_back({x, y, z});
                }
            }

            // Add the end point of the segment
            interpolated.push_back(end);
        }

        return interpolated;
    }

    std_msgs::msg::Float32MultiArray create_path_message(const std::vector<std::vector<double>>& points)
    {
        std_msgs::msg::Float32MultiArray msg;

        if (points.empty())
        {
            return msg;
        }

        // Set up the layout
        size_t coord_size = points[0].size();  // Should be 3 for [x, y, z]
        
        std_msgs::msg::MultiArrayDimension dim1;
        dim1.label = "points";
        dim1.size = points.size();
        dim1.stride = points.size() * coord_size;
        
        std_msgs::msg::MultiArrayDimension dim2;
        dim2.label = "coordinates";
        dim2.size = coord_size;
        dim2.stride = coord_size;
        
        msg.layout.dim.push_back(dim1);
        msg.layout.dim.push_back(dim2);
        msg.layout.data_offset = 0;

        // Flatten the points
        msg.data.reserve(points.size() * coord_size);
        for (const auto& point : points)
        {
            for (const auto& coord : point)
            {
                msg.data.push_back(static_cast<float>(coord));
            }
        }

        return msg;
    }

    void publish_paths()
    {
        if (waypoints_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No waypoints to publish");
            return;
        }

        // Publish raw path (without origin prepended)
        auto raw_msg = create_path_message(waypoints_);
        raw_path_publisher_->publish(raw_msg);
        //RCLCPP_INFO(this->get_logger(), "Published raw path with %lu points", waypoints_.size());

        // Interpolate and publish (with origin prepended)
        auto interpolated_points = interpolate_path(waypoints_);
        auto interpolated_msg = create_path_message(interpolated_points);
        interpolated_path_publisher_->publish(interpolated_msg);
        //RCLCPP_INFO(this->get_logger(), "Published interpolated path with %lu points (including origin)", 
                   //interpolated_points.size());
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr raw_path_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr interpolated_path_publisher_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Member variables
    std::vector<std::vector<double>> waypoints_;
    double interpolation_distance_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}
