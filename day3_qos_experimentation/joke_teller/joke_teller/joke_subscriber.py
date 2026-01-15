import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import(
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)

import time

class JokeSubscriber(Node):

    def __init__(self):
        super().__init__('joke_subscriber')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.subscription = self.create_subscription(
            String,
            '/programming_jokes',
            self.joke_callback,
            qos_profile
        )

        self.get_logger().info("Joke Subscriber Ready")

    def joke_callback(self,msg):
        time.sleep(0.2)
        self.get_logger().info(f"Recieved joke: {msg.data}")
        print("This is hilarious!")

def main(args=None):
    rclpy.init(args=args)
    node = JokeSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()