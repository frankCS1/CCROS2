#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(
            String,
            '/object_counts',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Control node started and monitoring object counts.')

    def listener_callback(self, msg):
        data = msg.data.lower()
        twist = Twist()

        if 'stop' in data:
            twist.linear.x = 0.0
            self.get_logger().info('Action: STOP')
        elif 'slow' in data:
            twist.linear.x = 0.05
            self.get_logger().info('Action: SLOW')
        elif 'speed' in data:
            twist.linear.x = 0.15
            self.get_logger().info('Action: SPEED UP')
        else:
            twist.linear.x = 0.1  # Default speed
            self.get_logger().info('Action: DEFAULT SPEED')

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down control node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
