#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from challenge_interfaces.msg import PathPoint
import math

class FSMOpenLoopPathController(Node):
    def __init__(self):
        super().__init__('fsm_open_loop_path_ctrl_node')

        self.path_sub = self.create_subscription(PathPoint, 'path_points', self.path_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_pub = self.create_publisher(PoseStamped, 'target_pose', 10)

        self.state = 'idle'  # FSM: 'idle', 'executing'
        self.cmd = Twist()
        self.start_time = None
        self.duration = 0.0
        self.path_points = []
        self.current_index = 0

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Controlador open-loop con máquina de estados iniciado.")

    def path_callback(self, msg):
        self.path_points.append(msg)
        self.get_logger().info(f"Recibido punto t={msg.time:.2f}s")

        marker = PoseStamped()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.pose = msg.pose
        self.target_pub.publish(marker)

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9 if self.start_time else 0.0

        if self.state == 'idle' and self.current_index < len(self.path_points) - 1:
            point = self.path_points[self.current_index]
            next_point = self.path_points[self.current_index + 1]

            self.cmd = point.velocity
            self.duration = next_point.time - point.time
            self.start_time = now
            self.state = 'executing'

            self.get_logger().info(
                f"[FSM] Ejecutando punto {self.current_index} durante {self.duration:.2f} s:"
                f" V_lin=({point.velocity.linear.x:.2f}), V_ang={point.velocity.angular.z:.2f}"
            )

        elif self.state == 'executing':
            if elapsed >= self.duration:
                self.cmd_vel_pub.publish(Twist())
                self.state = 'idle'
                self.current_index += 1
                self.get_logger().info("[FSM] Punto completado. Pasando al siguiente.")
            else:
                self.cmd_vel_pub.publish(self.cmd)

        elif self.current_index >= len(self.path_points):
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info("[FSM] Trayectoria completada.")

def main(args=None):
    rclpy.init(args=args)
    node = FSMOpenLoopPathController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_vel_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()