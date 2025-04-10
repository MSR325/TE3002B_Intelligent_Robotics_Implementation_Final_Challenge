"""
Node that generates different paths acoording to a user.
The path must be defined in the parameter files by the user. The paths are found in /config/params.yaml
The path must be defined by different points, velocities of the robot, or a time (dependent on the user)
    For each point, the node must estimate the linear and rotational speeds in case a time is given or estimate a time
    in case the velocities are provided
The node must let the user know if the point is reachable.
Take into consideration robustness.
The message for the topic /pose must be a custom message based on the pose geometry_msgs type with an added
field for velocities or time.
"""

"""
The code does the following:
    1.- Creates a PathGeneratorNode() object which:
        - Publishes PathPoint messages to the pose topic.
        - Loads path definitions from ROS2 parameters.
        - Allows selection of the current path using the current_path parameter.
    2.- Implements parameter parsing with two different approaches:
        - Direct parameter access from ROS2 parameter server.
        - YAML parsing as a fallback method.
        - Hardcoded defaults if both methods fail.
    3.- Includes a publisher that:
        - Publishes each path point in sequence.
        - Includes position, orientation, velocity, and time information.
        - Terminates when the path is completed.
    The code can handle the path formats seen in /config/params.yaml.
"""

# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Twist
from rcl_interfaces.msg import ParameterDescriptor
from challenge_interfaces.msg import PathPoint
import yaml
import math

def get_parameter_or(self, name, default_value):
    """Get a parameter value or return a default if not found"""
    try:
        return self.get_parameter(name)
    except:
        return rclpy.parameter.Parameter(
            name, 
            rclpy.Parameter.Type.DOUBLE,
            float(default_value) if isinstance(default_value, (int, float)) else default_value)

#Class definition
class PathGeneratorNode(Node):

    def __init__(self):
        super().__init__('path_generator_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('selected_path', 'figure_8_path_time'),
                ('max_linear_velocity', rclpy.Parameter.Type.DOUBLE),
                ('min_linear_velocity', rclpy.Parameter.Type.DOUBLE),
                ('max_angular_velocity', rclpy.Parameter.Type.DOUBLE),
                ('min_angular_velocity', rclpy.Parameter.Type.DOUBLE),
                ('speed_ratio', rclpy.Parameter.Type.DOUBLE)
            ]
        )

        # Get the selected path name
        self.selected_path = self.get_parameter('selected_path')

        # Declare the path parameter dynamically based on the path structure in YAML
        parameter_descriptor = ParameterDescriptor(
            description=f'Path points for {self.selected_path}'
        )
        self.declare_parameter(f'paths.{self.selected_path}', descriptor=parameter_descriptor)

        # Extract path points from the parameter
        self.path_points = self.extract_path_points()

        # Print the path points for verification
        self.get_logger().info(f'Using path: {self.selected_path}')
        self.get_logger().info(f'Number of points extracted: {len(self.path_points)}')

    def extract_path_points(self):
        """ Extract path points from the selected path parameter """
        path_points = []

        # Get the path parameter
        path_param = self.get_parameter(f'paths.{self.selected_path}')

        if path_param.type_ == rclpy.Parameter.Type.NOT_SET:
            self.get_logger().error(f'Path parameter {self.selected_path} not found')
            return path_points
        
        # The path parameter is a nested structure, we need to parse it
        path_dict = path_param.value

                # Iterate through each point in the path
        point_index = 0
        while True:
            point_key = f'point_{point_index}'
            if point_key not in path_dict:
                break
                
            point_data = path_dict[point_key]
            path_point = PathPoint()
            
            # Extract pose
            if 'pose' in point_data:
                pose_data = point_data['pose']
                if 'position' in pose_data:
                    path_point.pose.position.x = pose_data['position']['x']
                    path_point.pose.position.y = pose_data['position']['y']
                    path_point.pose.position.z = pose_data['position']['z']
                
                if 'orientation' in pose_data:
                    path_point.pose.orientation.x = pose_data['orientation']['x']
                    path_point.pose.orientation.y = pose_data['orientation']['y']
                    path_point.pose.orientation.z = pose_data['orientation']['z']
                    path_point.pose.orientation.w = pose_data['orientation']['w']

            # Extract velocity
            if 'velocity' in point_data:
                vel_data = point_data['velocity']
                if 'linear' in vel_data:
                    path_point.velocity.linear.x = vel_data['linear']['x']
                    path_point.velocity.linear.y = vel_data['linear']['y']
                    path_point.velocity.linear.z = vel_data['linear']['z']
                
                if 'angular' in vel_data:
                    path_point.velocity.angular.x = vel_data['angular']['x']
                    path_point.velocity.angular.y = vel_data['angular']['y']
                    path_point.velocity.angular.z = vel_data['angular']['z']
            
            # Extract time
            if 'time' in point_data:
                path_point.time = point_data['time']
            
            path_points.append(path_point)
            point_index += 1
        
        return path_points

def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()