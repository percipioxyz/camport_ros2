#!/usr/bin/env python3
"""
reset.py
ROS2 client example for resetting Percipio camera - Simplified version

Usage:
    python3 reset.py [camera_name]
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

def reset_camera(camera_name="camera"):
    """
    Send reset signals to specified camera
    
    Args:
        camera_name: Camera name
    """
    try:
        # Initialize ROS2
        rclpy.init()
        
        # Create node
        node = Node('device_reset_client')
        
        topic_name = f"/{camera_name}/reset"
        
        # Create publisher
        reset_pub = node.create_publisher(Empty, topic_name, 5)
        
        # Wait a moment for publisher to initialize
        node.get_logger().info(f"Publishing to: {topic_name}")
        
        # Wait for publisher to be ready (optional)
        import time
        time.sleep(0.5)
        
        # Create and publish empty message
        reset_msg = Empty()
        reset_pub.publish(reset_msg)
        
        # Spin once to ensure publication
        rclpy.spin_once(node, timeout_sec=0.1)
        
        node.get_logger().info("✓ Successfully sent reset signal")
        
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        
        return True
        
    except Exception as e:
        print(f"Error sending reset signal: {e}")
        return False

if __name__ == "__main__":
    # Get camera name from command line arguments
    camera_name = "camera"  # Default value
    
    if len(sys.argv) > 1:
        camera_name = sys.argv[1]
    
    print(f"=== ROS2 Percipio Camera Reset Client ===")
    print(f"Camera name: {camera_name}")
    reset_camera(camera_name)