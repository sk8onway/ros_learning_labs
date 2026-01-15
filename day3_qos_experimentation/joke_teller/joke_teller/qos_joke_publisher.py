import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)

class QoSJokePublisher(Node):

    def __init__(self):
        super().__init__('qos_joke_publisher')

        #CHANGE QoS here for experiments
        qos_profile = QoSProfile(
            depth = 10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(
            String,
            '/programming_jokes',
            qos_profile
        )

        self.timer = self.create_timer(0.01, self.publish_joke)
        self.counter = 0

        self.get_logger().info("QoS Joke Publisher Started")
    
    def publish_joke(self):
        msg = String()
        msg.data = f"Joke #{self.counter}"
        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Published: {msg.data}"
        )

        self.counter+=1

def main(args=None):
    rclpy.init(args=args)
    node = QoSJokePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()