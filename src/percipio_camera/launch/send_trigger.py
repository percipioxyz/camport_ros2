import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 
class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, '/camera/event', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0
 
    def timer_callback(self):
        msg = String()
        msg.data = 'SoftTrigger:%d' % self.counter
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)
        self.counter += 1
 
def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
