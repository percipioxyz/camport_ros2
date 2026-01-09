#!/usr/bin/env python3
#-*- coding: utf-8 -*-
"""
Percipio Camera Dynamic Configuration Client - ROS2 Version (Simplified)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time

def configure_camera_simple(camera_name: str = "camera") -> None:
    """
    Simplified version of camera configuration without a separate class.
    
    Args:
        camera_name: ROS2 namespace for the camera device
    """
    #Initialize ROS2
    rclpy.init()
    
    #Create a minimal node
    node = Node('camera_configurator')
    
    #Construct topic name and create publisher
    topic_name = f"/{camera_name}/dynamic_config"
    node.get_logger().info(f"Publishing to topic: {topic_name}")
    publisher = node.create_publisher(String, topic_name, 10)
    
    #Wait for publisher to establish connection
    time.sleep(1)
    
    #XML configuration
    xml_config = '''<source name="Texture">
        <feature name="ExposureAuto">1</feature>
        <feature name="AutoFunctionAOIOffsetX">10</feature>
        <feature name="AutoFunctionAOIOffsetY">10</feature>
        <feature name="AutoFunctionAOIWidth">30</feature>
        <feature name="AutoFunctionAOIHeight">30</feature>
    </source>'''
    
    #Create and publish message
    config_msg = String()
    config_msg.data = xml_config
    publisher.publish(config_msg)
    
    node.get_logger().info("XML configuration published successfully")
    
    #Clean up
    time.sleep(0.5)  #Ensure message is sent
    node.destroy_node()
    rclpy.shutdown()

def main_simple() -> None:
    """
    Main function for the simplified version.
    """
    #Parse command line arguments
    #Default camera name if no command-line argument provided
    camera_name = "camera"
    
    #Parse command line arguments
    #sys.argv[0] is the script name, sys.argv[1] would be the first argument
    if len(sys.argv) > 1:
        camera_name = sys.argv[1]
    
    print("=" * 50)
    print("Percipio Camera XML Configuration Client - ROS2 (Simplified)")
    print("=" * 50)
    
    try:
        configure_camera_simple(camera_name)
        print("Configuration completed successfully")
    except KeyboardInterrupt:
        print("\nConfiguration interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    print("=" * 50)


if __name__ == "__main__":
    #Uncomment to use the simplified version
    #main_simple()
    
    #Or use the class-based version
    main_simple()