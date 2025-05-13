#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TrafficNode(Node):
    def __init__(self):
        super().__init__('traffic_node')
        self.subscription = self.create_subscription(
            String,
            '/object_counts',
            self.listener_callback,
            10
        )
        self.get_logger().info('Traffic node started and listening for sign detections.')

    def listener_callback(self, msg):
        data = msg.data.lower()
        action = None

        if 'stop' in data:
            action = 'STOP the robot.'
        elif 'slow' in data:
            action = 'Reduce speed.'
        elif 'speed' in data:
            action = 'Increase speed.'
        else:
            action = 'No speed change required.'

        self.get_logger().info(f"Traffic Sign Action: {action}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down traffic node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
