#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_node')
        self.subscription = self.create_subscription(
            String,
            '/object_counts',
            self.detection_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_reached = False
        self.get_logger().info('Goal node started, watching for goal marker...')

    def detection_callback(self, msg):
        if self.goal_reached:
            return  # Stop processing once stopped

        if 'goal' in msg.data.lower():  # or any keyword like "flag" or "marker"
            self.get_logger().info('Goal marker detected! Stopping the robot.')
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_pub.publish(stop_cmd)
            self.goal_reached = True
        else:
            self.get_logger().info('Goal not detected yet.')

def main(args=None):
    rclpy.init(args=args)
    node = GoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down goal node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
