import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("Simple_subscriber")
        self.sub = self.create_subscription(String, "Chatter", self.MsgCallback, 10)
    def MsgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)
   
def main():
    rclpy.init()
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__name__':
    main()