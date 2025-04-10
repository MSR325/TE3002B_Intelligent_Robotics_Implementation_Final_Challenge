#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from challenge_interfaces.msg import PathPoint
import yaml
import math

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_node')

        # Publisher for PathPoint messages
        self.path_pub = self.create_publisher(PathPoint, 'path_points', 10)

        # Get the selected path name from the launch argument
        self.declare_parameter('selected_path', 'figure_8_path_time')
        self.selected_path = self.get_parameter('selected_path').value
        
        # Load the entire params.yaml
        self.declare_parameter('params_file', '/home/chrisrvt/projects/Implementation_of_Intelligent_Robotics/challenge/src/puzzlebot_control/config/params.yaml')
        params_file = self.get_parameter('params_file').value
        
        if not params_file:
            self.get_logger().error('No params file specified!')
            return
        
        # Load YAML file to process path
        with open(params_file, 'r') as f:
            self.params = yaml.safe_load(f)['path_generator_node']['ros__parameters']['paths']
        
        # Extract the selected path
        if self.selected_path not in self.params:
            self.get_logger().error(f'Path {self.selected_path} not found!')
            return
        
        self.path_points = self.params[self.selected_path]
        processed_points = self.process_path(self.path_points)
        self.publish_path(processed_points)

    def process_path(self, path_data):
        """Process path and calculate missing values"""

        if self.selected_path.endswith('_time'):
            self.get_logger().info(f'Processing TIME-BASED path: {self.selected_path}')
            # Compute velocities here if needed
        elif self.selected_path.endswith('_speed'):
            self.get_logger().info(f'Processing SPEED-BASED path: {self.selected_path}')
            # Compute timestamps here if needed
        
        # Log all points (for debugging)
        for point_name, point_data in self.path_points.items():
            self.get_logger().info(
                f"{point_name}: Pose={point_data['pose']}, "
                f"Velocity={point_data['velocity']}, Time={point_data['time']}"
            )

        processed_points = []
        points = list(path_data.values())
        
        if self.selected_path.endswith('_time'):
            # Calculate velocities between points (time defined)
            for i in range(len(points)-1):
                current = points[i]
                next_pt = points[i+1]
                
                # Linear velocity calculation
                dx = next_pt['pose']['position']['x'] - current['pose']['position']['x']
                dy = next_pt['pose']['position']['y'] - current['pose']['position']['y']
                distance = math.sqrt(dx**2 + dy**2)
                dt = next_pt['time'] - current['time']
                
                linear_vel = distance / dt if dt > 0 else 0.0
                current_yaw = self.quaternion_to_yaw(current['pose']['orientation'])
                linear_x = linear_vel * math.cos(current_yaw)
                linear_y = linear_vel * math.sin(current_yaw)
                
                # Angular velocity calculation
                next_yaw = self.quaternion_to_yaw(next_pt['pose']['orientation'])
                yaw_diff = self.normalize_angle(next_yaw - current_yaw)
                angular_vel = yaw_diff / dt if dt > 0 else 0.0
                
                # Create PathPoint message
                path_point = PathPoint()
                path_point.pose = self.dict_to_pose(current['pose'])
                path_point.velocity.linear.x = linear_x
                path_point.velocity.linear.y = linear_y
                path_point.velocity.angular.z = angular_vel
                path_point.time = current['time']
                
                processed_points.append(path_point)
            
            # Add last point
            last_point = PathPoint()
            last_point.pose = self.dict_to_pose(points[-1]['pose'])
            last_point.time = points[-1]['time']
            processed_points.append(last_point)
            
        elif self.selected_path.endswith('_speed'):
            # Calculate times between points (velocity defined)
            current_time = 0.0
            
            # First point
            first_point = PathPoint()
            first_point.pose = self.dict_to_pose(points[0]['pose'])
            first_point.velocity = self.dict_to_twist(points[0]['velocity'])
            first_point.time = current_time
            processed_points.append(first_point)
            
            for i in range(len(points)-1):
                current = points[i]
                next_pt = points[i+1]
                
                # Calculate required time
                dx = next_pt['pose']['position']['x'] - current['pose']['position']['x']
                dy = next_pt['pose']['position']['y'] - current['pose']['position']['y']
                distance = math.sqrt(dx**2 + dy**2)
                
                linear_speed = math.sqrt(current['velocity']['linear']['x']**2 + 
                                current['velocity']['linear']['y']**2)
                dt_linear = distance / linear_speed if linear_speed > 0 else 0.0
                
                current_yaw = self.quaternion_to_yaw(current['pose']['orientation'])
                next_yaw = self.quaternion_to_yaw(next_pt['pose']['orientation'])
                yaw_diff = abs(self.normalize_angle(next_yaw - current_yaw))
                angular_speed = abs(current['velocity']['angular']['z'])
                dt_angular = yaw_diff / angular_speed if angular_speed > 0 else 0.0
                
                dt = max(dt_linear, dt_angular)
                current_time += dt
                
                # Create next PathPoint
                path_point = PathPoint()
                path_point.pose = self.dict_to_pose(next_pt['pose'])
                path_point.velocity = self.dict_to_twist(next_pt['velocity'])
                path_point.time = current_time
                processed_points.append(path_point)
        
        return processed_points

    def publish_path(self, path_points):
        """Publish processed path as PathPoint messages"""
        for point in path_points:
            self.path_pub.publish(point)
            self.get_logger().info(
                f"Published Point at {point.time:.2f}s:\n"
                f"  Position: [{point.pose.position.x:.2f}, {point.pose.position.y:.2f}]\n"
                f"  Orientation: [{point.pose.orientation.z:.2f} rad]\n"
                f"  Linear Vel: [{point.velocity.linear.x:.2f}, {point.velocity.linear.y:.2f}] m/s\n"
                f"  Angular Vel: {point.velocity.angular.z:.2f} rad/s\n"
            )
            rclpy.spin_once(self, timeout_sec=0.1)

    # Helper functions
    def dict_to_pose(self, pose_dict):
        pose = Pose()
        pose.position.x = pose_dict['position']['x']
        pose.position.y = pose_dict['position']['y']
        pose.position.z = pose_dict['position']['z']
        pose.orientation.x = pose_dict['orientation']['x']
        pose.orientation.y = pose_dict['orientation']['y']
        pose.orientation.z = pose_dict['orientation']['z']
        pose.orientation.w = pose_dict['orientation']['w']
        return pose

    def dict_to_twist(self, twist_dict):
        twist = Twist()
        twist.linear.x = twist_dict['linear']['x']
        twist.linear.y = twist_dict['linear']['y']
        twist.linear.z = twist_dict['linear']['z']
        twist.angular.x = twist_dict['angular']['x']
        twist.angular.y = twist_dict['angular']['y']
        twist.angular.z = twist_dict['angular']['z']
        return twist

    def quaternion_to_yaw(self, quat_dict):
        x = quat_dict['x']
        y = quat_dict['y']
        z = quat_dict['z']
        w = quat_dict['w']
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()