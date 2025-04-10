import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from challenge_interfaces.msg import PathPoint
from rcl_interfaces.msg import ParameterDescriptor
import math
from collections import deque

class OpenLoopPathCtrlNode(Node):
    def __init__(self):
        super().__init__('open_loop_path_ctrl')
        
        # Publisher to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber to /pose (from path_generator_node)
        self.pose_sub = self.create_subscription(
            PathPoint, 'pose', self.path_point_callback, 10)
        
        # Parameters
        self.declare_parameter('lookahead_time', 0.2, 
                               ParameterDescriptor(description='Time to look ahead on the path'))
        self.declare_parameter('control_frequency', 20.0, 
                               ParameterDescriptor(description='Control loop frequency in Hz'))
        
        # Path storage
        self.path_points = deque()
        self.current_target = None
        self.next_target = None
        self.path_started = False
        self.path_complete = False
        
        # Time tracking
        self.start_time = None
        
        # Control loop timer
        control_period = 1.0 / self.get_parameter('control_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(control_period, self.control_loop)
        
        self.get_logger().info("Trajectory follower initialized and waiting for path points...")

    def path_point_callback(self, msg):
        """Store incoming path points from the path generator"""
        self.path_points.append(msg)
        self.get_logger().debug(f"Received path point: pos=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), "
                               f"time={msg.time:.2f}")
        
        # Initialize path following when we get the first point
        if not self.path_started and len(self.path_points) > 0:
            self.current_target = self.path_points.popleft()
            self.start_time = self.get_clock().now()
            self.path_started = True
            self.get_logger().info("Path following started!")

    def control_loop(self):
        """Main control loop for following the trajectory"""
        if not self.path_started or self.path_complete:
            # Nothing to do yet or we're done
            return
            
        # Calculate elapsed time since path start
        now = self.get_clock().now()
        elapsed_seconds = (now - self.start_time).nanoseconds * 1e-9
        
        # Use the velocity commands from the current target point
        cmd = Twist()
        cmd.linear.x = self.current_target.velocity.linear.x
        cmd.angular.z = self.current_target.velocity.angular.z
        
        # Check if it's time to move to the next point
        if len(self.path_points) > 0:
            next_point = self.path_points[0]  # Peek at next point
            
            if elapsed_seconds >= next_point.time:
                # Time to move to next point
                self.current_target = self.path_points.popleft()
                self.get_logger().info(f"Moving to next path point: "
                                     f"pos=({self.current_target.pose.position.x:.2f}, "
                                     f"{self.current_target.pose.position.y:.2f}), "
                                     f"time={self.current_target.time:.2f}")
                
                # Update velocities
                cmd.linear.x = self.current_target.velocity.linear.x
                cmd.angular.z = self.current_target.velocity.angular.z
        
        # Check if we've reached the end of the path
        elif elapsed_seconds >= self.current_target.time:
            # We've completed the path
            self.get_logger().info("Path completed!")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.path_complete = True
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug(f"Published velocity: linear={cmd.linear.x:.3f}, angular={cmd.angular.z:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopPathCtrlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Trajectory following interrupted by user")
    finally:
        # Ensure robot stops when node is terminated
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()