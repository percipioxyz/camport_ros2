import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
class StringSubscriber(Node):
    def __init__(self):
        super().__init__('string_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/camera/event',
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