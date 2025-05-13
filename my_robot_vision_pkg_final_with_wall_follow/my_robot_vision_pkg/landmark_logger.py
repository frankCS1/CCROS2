#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
from datetime import datetime

class LandmarkLogger(Node):
    def __init__(self):
        super().__init__('landmark_logger')
        self.subscription = self.create_subscription(
            String,
            '/object_counts',
            self.log_callback,
            10
        )
        self.landmark_data = {'orange': 0, 'tree': 0, 'vehicle': 0}
        self.file_path = '/tmp/landmark_log.csv'
        self.init_csv()
        self.get_logger().info('Landmark logger started.')

    def init_csv(self):
        with open(self.file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'Orange', 'Tree', 'Vehicle'])

    def log_callback(self, msg):
        counts = msg.data.lower()
        for key in self.landmark_data.keys():
            for part in counts.split(','):
                if key in part:
                    try:
                        value = int(part.strip().split(':')[1])
                        self.landmark_data[key] = value
                    except (IndexError, ValueError):
                        pass

        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with open(self.file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                timestamp,
                self.landmark_data['orange'],
                self.landmark_data['tree'],
                self.landmark_data['vehicle']
            ])
        self.get_logger().info(f"Logged landmark data: {self.landmark_data}")

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down landmark logger.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
