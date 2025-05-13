#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Example goal (hardcoded for now)
        self.goal_x = 2.0
        self.goal_y = 2.0
        self.current_x = 0.0
        self.current_y = 0.0

        self.get_logger().info('Navigation node with basic A* logic started.')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.hypot(dx, dy)

        cmd = Twist()

        if distance > 0.1:
            angle_to_goal = math.atan2(dy, dx)
            cmd.linear.x = 0.1
            cmd.angular.z = angle_to_goal
            self.get_logger().info(f'Navigating to ({self.goal_x}, {self.goal_y})')
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Reached goal!')

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down nav node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
