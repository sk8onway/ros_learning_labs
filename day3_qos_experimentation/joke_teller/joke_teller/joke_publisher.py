import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class JokePublisher(Node):
    
    def __init__(self):
        super().__init__('joke_publisher')

        self.publisher_ = self.create_publisher(
            String, 
            '/programming_jokes', 
            10
        )

        self.jokes = [
            "Why do programmers prefer dark mode? Because light attracts bugs",
            "I told my computer I needed a break, and it froze.",
            "There are only 10 kinds of people: those who understand binary and those who don’t.",
            "Why did the programmer quit his job? Because he didn’t get arrays.",
            "Debugging: Being the detective in a crime movie where you are also the murderer."
        ]

        self.timer = self.create_timer(5.0, self.publish_joke)
        self.get_logger().info("Joke Publisher started")

    def publish_joke(self):
        msg = String()
        msg.data = random.choice(self.jokes)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Joke sent: {msg.data}")
    
def main(args=None):
    rclpy.init(args=args)
    node = JokePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()