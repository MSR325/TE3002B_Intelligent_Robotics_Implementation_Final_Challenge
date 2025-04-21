#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from challenge_interfaces.msg import PathPoint
import yaml
import math

class SegmentedPathGenerator(Node):
    def __init__(self):
        super().__init__('segmented_path_generator_node')
        self.path_pub = self.create_publisher(PathPoint, 'path_points', 10)

        self.declare_parameter('selected_path', 'square_path_time')
        self.selected_path = self.get_parameter('selected_path').value

        self.declare_parameter('params_file', '/path/to/params.yaml')
        params_file = self.get_parameter('params_file').value

        if not params_file:
            self.get_logger().error('No params file specified!')
            return

        with open(params_file, 'r') as f:
            self.params = yaml.safe_load(f)['path_generator_node']['ros__parameters']['paths']

        if self.selected_path not in self.params:
            self.get_logger().error(f'Path {self.selected_path} not found!')
            return

        self.path_points = self.params[self.selected_path]
        self.processed_points = self.generate_segmented_path(self.path_points)

        self.current_point_index = 0
        self.path_start_time = None

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if not self.processed_points or self.current_point_index >= len(self.processed_points):
            return

        if self.path_start_time is None:
            self.path_start_time = self.get_clock().now().nanoseconds / 1e9
            return

        current_time = self.get_clock().now().nanoseconds / 1e9 - self.path_start_time
        next_point = self.processed_points[self.current_point_index]

        if current_time >= next_point.time:
            self.path_pub.publish(next_point)
            self.get_logger().info(f"Publicado punto {self.current_point_index} a t={next_point.time:.2f}s")
            self.current_point_index += 1

    def generate_segmented_path(self, path_data):
        FIXED_LINEAR_VEL = 0.2
        FIXED_ANGULAR_VEL = 1.0
        MIN_ANGULAR_VEL = 0.3

        points = list(path_data.values())
        current_time = 0.0
        processed_points = []

        for i in range(len(points)-1):
            current = points[i]
            next_pt = points[i+1]

            x1, y1 = current['pose']['position']['x'], current['pose']['position']['y']
            x2, y2 = next_pt['pose']['position']['x'], next_pt['pose']['position']['y']

            yaw1 = self.quaternion_to_yaw(current['pose']['orientation'])
            yaw2 = self.quaternion_to_yaw(next_pt['pose']['orientation'])

            yaw_diff = self.normalize_angle(yaw2 - yaw1)
            dx, dy = x2 - x1, y2 - y1
            distance = math.hypot(dx, dy)

            angular_speed = max(abs(yaw_diff) / 1.0, MIN_ANGULAR_VEL)
            if angular_speed > FIXED_ANGULAR_VEL:
                self.get_logger().warn(f"Velocidad angular ajustada: {angular_speed:.2f} -> {FIXED_ANGULAR_VEL:.2f} rad/s")
                angular_speed = FIXED_ANGULAR_VEL

            linear_speed = FIXED_LINEAR_VEL

            # Punto 1: orientación
            orient_point = PathPoint()
            orient_point.pose = self.dict_to_pose(current['pose'])
            orient_point.velocity.angular.z = angular_speed if yaw_diff >= 0 else -angular_speed
            orient_point.time = current_time
            processed_points.append(orient_point)

            current_time += abs(yaw_diff) / angular_speed if angular_speed > 0 else 0.0

            # Punto 2: avance
            move_point = PathPoint()
            move_point.pose = self.dict_to_pose(current['pose'])  # misma pose de inicio
            move_point.velocity.linear.x = linear_speed
            move_point.time = current_time
            processed_points.append(move_point)

            current_time += distance / linear_speed if linear_speed > 0 else 0.0

        # Último punto (solo como final)
        last = PathPoint()
        last.pose = self.dict_to_pose(points[-1]['pose'])
        last.time = current_time
        processed_points.append(last)

        return processed_points

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

    def quaternion_to_yaw(self, quat):
        x, y, z, w = quat['x'], quat['y'], quat['z'], quat['w']
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
    node = SegmentedPathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
