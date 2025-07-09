import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import yaml
import os
import numpy as np
from math import sqrt

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('interpolation_distance', 0.01)  # Distance between waypoints in meters

        config_file = self.get_parameter('config_file').value
        self.interpolation_distance = self.get_parameter('interpolation_distance').value

        self.get_logger().info(f'Config file path: {config_file}')
        self.get_logger().info(f'Interpolation distance: {self.interpolation_distance} meters')

        # Create publishers
        self.raw_path_publisher = self.create_publisher(Float32MultiArray, 'raw_path', 10)
        self.interpolated_path_publisher = self.create_publisher(Float32MultiArray, '/path', 10)

        # Load waypoints from file
        if not config_file:
            self.get_logger().error('No config file specified. Use --ros-args -p config_file:=<path>')
            self.waypoints = []
        else:
            self.waypoints = self.load_waypoints_from_file(config_file)

        if self.waypoints:
            # Publish both raw and interpolated paths immediately
            self.publish_paths()

            # Set up a timer to periodically publish the paths
            self.timer = self.create_timer(1.0, self.publish_paths)  # Publish every 1 second

    def load_waypoints_from_file(self, config_file):
        """Load waypoints from YAML file and return as list of points"""
        try:
            self.get_logger().info(f'Loading waypoints from {config_file}')

            if not os.path.exists(config_file):
                self.get_logger().error(f'Config file does not exist: {config_file}')
                return []

            with open(config_file, 'r') as f:
                data = yaml.safe_load(f)

            if not data or 'path_data' not in data:
                self.get_logger().error('Invalid YAML: missing path_data section')
                return []

            path_data = data['path_data']

            if 'waypoints' not in path_data:
                self.get_logger().error('Invalid YAML: missing waypoints section')
                return []

            waypoints_data = path_data['waypoints']
            waypoints = []

            # Convert waypoints based on format
            if isinstance(waypoints_data, list):
                if waypoints_data and isinstance(waypoints_data[0], list):
                    # Format: [[x, y, z], [x, y, z], ...]
                    waypoints = waypoints_data
                elif waypoints_data and isinstance(waypoints_data[0], dict):
                    # Format: [{x: val, y: val, z: val}, ...]
                    for wp in waypoints_data:
                        if all(k in wp for k in ['x', 'y', 'z']):
                            waypoints.append([wp['x'], wp['y'], wp['z']])

            self.get_logger().info(f'Loaded {len(waypoints)} waypoints')
            return waypoints

        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {str(e)}')
            return []

    def interpolate_path(self, points):
        """Interpolate the path to have points every interpolation_distance meters"""
        if len(points) == 0:
            return np.array([[0, 0, 0]])  # Return origin if no points

        # Convert to numpy array and ensure 3D coordinates
        points = np.array(points)
        if points.ndim == 1:
            points = points.reshape(-1, 3) if len(points) > 2 else points.reshape(-1, 2)

        # Prepend origin point (0,0,0) or (0,0) depending on input dimension
        origin = np.zeros(3) if points.shape[1] > 2 else np.zeros(2)
        points_with_origin = np.vstack([origin, points])

        interpolated = [points_with_origin[0]]  # Start with origin

        for i in range(len(points_with_origin) - 1):
            start = points_with_origin[i]
            end = points_with_origin[i + 1]

            # Calculate distance between current segment points
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dz = end[2] - start[2] if len(start) > 2 else 0
            segment_length = sqrt(dx**2 + dy**2 + dz**2)

            if segment_length == 0:
                continue  # Skip zero-length segments

            # Calculate number of interpolation points needed
            num_points = int(segment_length / self.interpolation_distance)

            if num_points > 0:
                # Create linearly spaced points along the segment
                t = np.linspace(0, 1, num_points + 2)[1:-1]  # Exclude start and end points
                x = start[0] + t * dx
                y = start[1] + t * dy
                z = start[2] + t * dz if len(start) > 2 else np.zeros_like(t) * start[2]

                # Add the interpolated points
                for xi, yi, zi in zip(x, y, z):
                    if len(start) > 2:
                        interpolated.append([xi, yi, zi])
                    else:
                        interpolated.append([xi, yi])

            # Add the end point of the segment
            interpolated.append(end)

        return np.array(interpolated)

    def create_path_message(self, points):
        """Create a Float32MultiArray message from a list of points"""
        msg = Float32MultiArray()

        # Ensure points is a numpy array
        points_array = np.array(points)
        if points_array.ndim == 1:
            points_array = points_array.reshape(-1, 3) if len(points_array) > 2 else points_array.reshape(-1, 2)

        # Set up the layout
        coord_size = points_array.shape[1]  # 2 or 3 depending on dimensions
        msg.layout.dim = [
            MultiArrayDimension(
                label="points",
                size=len(points_array),
                stride=len(points_array) * coord_size
            ),
            MultiArrayDimension(
                label="coordinates",
                size=coord_size,
                stride=coord_size
            )
        ]
        msg.layout.data_offset = 0

        # Flatten the points
        msg.data = points_array.flatten().tolist()

        return msg

    def publish_paths(self):
        """Publish both raw and interpolated paths"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints to publish')
            return

        # Publish raw path (without origin prepended)
        raw_msg = self.create_path_message(self.waypoints)
        self.raw_path_publisher.publish(raw_msg)
        self.get_logger().info(f'Published raw path with {len(self.waypoints)} points')

        # Interpolate and publish (with origin prepended)
        interpolated_points = self.interpolate_path(self.waypoints)
        interpolated_msg = self.create_path_message(interpolated_points)
        self.interpolated_path_publisher.publish(interpolated_msg)
        self.get_logger().info(f'Published interpolated path with {len(interpolated_points)} points (including origin)')


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
