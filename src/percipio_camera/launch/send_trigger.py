import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

#This Python file demonstrates how to send soft trigger signals to the camera node.
#Note: The soft trigger signal will only take effect when the camera node sets the camera operation mode to soft trigger mode.
class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self.publisher_ = self.create_publisher(String, '/camera/soft_trigger', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
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
