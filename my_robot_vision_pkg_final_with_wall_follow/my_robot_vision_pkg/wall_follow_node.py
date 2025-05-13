#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # LaserScan input
            self.laser_callback,
            10
        )

        self.get_logger().info('wall_follow_node started — following wall.')

    def laser_callback(self, msg):
        # Define detection angles
        left = min(msg.ranges[90:110])  # Left side
        front = min(min(msg.ranges[0:20]), min(msg.ranges[340:359]))  # Front

        cmd = Twist()

        if front < 0.4:
            # Turn right if there's something ahead
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
            self.get_logger().info('Wall ahead — turning right.')
        elif left > 0.5:
            # No wall on left — turn left to reacquire
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3
            self.get_logger().info('Searching for left wall — turning left.')
        else:
            # Wall detected on left — move forward
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
            self.get_logger().info('Following wall.')

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down wall_follow_node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
