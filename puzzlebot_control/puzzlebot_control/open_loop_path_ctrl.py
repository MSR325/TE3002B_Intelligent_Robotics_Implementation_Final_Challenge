#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from challenge_interfaces.msg import PathPoint
import math

class PathController(Node):
    def __init__(self):
        super().__init__('open_loop_path_ctrl_node')
        
        # Subscriber to path points
        self.path_sub = self.create_subscription(
            PathPoint, 
            'path_points', 
            self.path_callback, 
            10)
            
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Publisher for visualization (optional)
        self.target_pub = self.create_publisher(PoseStamped, 'target_pose', 10)
        
        # Controller parameters
        self.declare_parameter('linear_tolerance', 0.05)  # meters
        self.declare_parameter('angular_tolerance', 0.1)  # radians
        self.declare_parameter('max_linear_vel', 0.5)     # m/s
        self.declare_parameter('max_angular_vel', 1.0)    # rad/s
        
        # Current target point
        self.current_target = None
        self.current_index = 0
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 20Hz
        
        self.get_logger().info("Path Controller initialized")

    def path_callback(self, msg):
        """Store the latest path point as target"""
        self.current_target = msg
        self.current_index += 1
        
        # Publish target pose for visualization
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "odom"
        target_pose.pose = msg.pose
        self.target_pub.publish(target_pose)
        
        self.get_logger().info(
            f"Received new target point {self.current_index}: "
            f"Position: [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}], "
            f"Velocity: [lin: {msg.velocity.linear.x:.2f}, ang: {msg.velocity.angular.z:.2f}]"
        )

    def control_loop(self):
        """Main control loop that publishes velocity commands"""
        if self.current_target is None:
            return
            
        # Create velocity command
        cmd_vel = Twist()
        
        # Open-loop control - simply use the commanded velocities
        # (For closed-loop, you would calculate errors here)
        cmd_vel.linear.x = self.current_target.velocity.linear.x
        cmd_vel.linear.y = self.current_target.velocity.linear.y
        cmd_vel.angular.z = self.current_target.velocity.angular.z
        
        # Apply velocity limits
        max_lin = self.get_parameter('max_linear_vel').value
        max_ang = self.get_parameter('max_angular_vel').value
        
        cmd_vel.linear.x = max(-max_lin, min(max_lin, cmd_vel.linear.x))
        cmd_vel.linear.y = max(-max_lin, min(max_lin, cmd_vel.linear.y))
        cmd_vel.angular.z = max(-max_ang, min(max_ang, cmd_vel.angular.z))
        
        self.get_logger().info(f"Linear velocity x: {cmd_vel.linear.x}")
        self.get_logger().info(f"Linear velocity y: {cmd_vel.linear.y}")
        self.get_logger().info(f"Angular velocity z: {cmd_vel.angular.z}")

        self.cmd_vel_pub.publish(cmd_vel)

    def stop_robot(self):
        """Stop the robot by publishing zero velocities"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info("Stopping robot")

def main(args=None):
    rclpy.init(args=args)
    
    controller = PathController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()