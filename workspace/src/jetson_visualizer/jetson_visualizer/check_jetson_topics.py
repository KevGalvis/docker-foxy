#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import os

class TopicChecker(Node):
    def __init__(self):
        super().__init__('topic_checker')
        self.get_logger().info('Checking for available topics...')
        self.timer = self.create_timer(2.0, self.check_topics)
        self.discovered_topics = set()
        
    def check_topics(self):
        topic_list = self.get_topic_names_and_types()
        
        # Display all topics
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('DISCOVERED TOPICS:')
        self.get_logger().info('='*50)
        
        for topic_name, topic_types in topic_list:
            if topic_name not in self.discovered_topics:
                self.discovered_topics.add(topic_name)
                self.get_logger().info(f'New topic: {topic_name} [{", ".join(topic_types)}]')
            
            # Check common topics from Jetson
            if topic_name in ['/scan', '/camera/image_raw', '/camera/camera_info', 
                              '/imu/data', '/odom', '/tf', '/tf_static']:
                self.get_logger().info(f'✓ Found essential topic: {topic_name} [{", ".join(topic_types)}]')
        
        self.get_logger().info('='*50)
        self.get_logger().info(f'Total topics: {len(topic_list)}')
        
        # Check network configuration
        rmw_impl = os.environ.get('RMW_IMPLEMENTATION', 'Not set')
        cyclone_uri = os.environ.get('CYCLONEDDS_URI', 'Not set')
        
        self.get_logger().info(f'RMW_IMPLEMENTATION: {rmw_impl}')
        self.get_logger().info(f'CYCLONEDDS_URI: {cyclone_uri}')
        self.get_logger().info('='*50)

def main():
    rclpy.init()
    checker = TopicChecker()
    
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()