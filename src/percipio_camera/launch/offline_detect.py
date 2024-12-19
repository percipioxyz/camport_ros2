import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
#This Python file demonstrates how to receive camera node offline events
#In addition, the camera capture image timeout is also captured here
#If the automatic camera disconnection function is enabled, 
#this event will also be captured here after the camera disconnection successfully completes the reconnection
class StringSubscriber(Node):
    def __init__(self):
        super().__init__('string_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/camera/device_event',
            self.listener_callback,
            10)
 
    def listener_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.data)
 
def main(args=None):
    rclpy.init(args=args)
    string_subscriber = StringSubscriber()
    rclpy.spin(string_subscriber)
    string_subscriber.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()