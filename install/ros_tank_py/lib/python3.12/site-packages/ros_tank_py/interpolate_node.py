import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from math import sqrt

class PathInterpolator(Node):
    def __init__(self):
        super().__init__('path_interpolator')
        
        # Parameters
        self.interpolation_distance = 0.1  # Distance between waypoints in meters
        
        # Subscriber for the original path
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'input_path',
            self.path_callback,
            10)
        
        # Publisher for the interpolated path
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/path',
            10)
        
        self.get_logger().info('Path Interpolator Node has been started')
    
    def path_callback(self, msg):
        # Extract the path points from the message
        points = np.array(msg.data).reshape(-1, 2)
        
        if len(points) < 2:
            self.get_logger().warn('Received path with less than 2 points')
            return
        
        # Interpolate the path
        interpolated_points = self.interpolate_path(points)
        
        # Create and publish the new message
        interpolated_msg = Float32MultiArray()
        interpolated_msg.data = interpolated_points.flatten().tolist()
        self.publisher.publish(interpolated_msg)
        
        self.get_logger().info(f'Published interpolated path with {len(interpolated_points)} points')
    
    def interpolate_path(self, points):
        """Interpolate the path to have points every interpolation_distance meters"""
        interpolated = [points[0]]  # Start with the first point
        
        for i in range(len(points) - 1):
            start = points[i]
            end = points[i + 1]
            
            # Calculate distance between current segment points
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            segment_length = sqrt(dx**2 + dy**2)
            
            if segment_length == 0:
                continue  # Skip zero-length segments
            
            # Calculate number of interpolation points needed
            num_points = int(segment_length / self.interpolation_distance)
            
            if num_points > 0:
                # Create linearly spaced points along the segment
                t = np.linspace(0, 1, num_points + 2)[1:-1]  # Exclude start and end points
                x = start[0] + t * dx
                y = start[1] + t * dy
                
                # Add the interpolated points
                for xi, yi in zip(x, y):
                    interpolated.append([xi, yi])
            
            # Add the end point of the segment
            interpolated.append(end)
        
        return np.array(interpolated)


def main(args=None):
    rclpy.init(args=args)
    node = PathInterpolator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
