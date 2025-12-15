import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberPublisher(Node):
	def __init__(self):
		super().__init__('number_publisher')
		self.publisher_ = self.create_publisher(Int32,'numbers',10)
		self.counter_ = 0
		self.timer_ = self.create_timer(1.0, self.timer_callback)
	def timer_callback(self):
		msg=Int32()
		msg.data = self.counter_
		self.publisher_.publish(msg)
		self.get_logger().info(f'Publishing: {msg.data}')
		self.counter_+=1

def main(args=None):
	rclpy.init(args=args)
	node = NumberPublisher()
	rclpy.spin(node)
	node.destory_node()
	rclpy.shutdown()
