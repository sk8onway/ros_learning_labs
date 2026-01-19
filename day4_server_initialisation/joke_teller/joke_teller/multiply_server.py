import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class MultiplyServer(Node):

    def __init__(self):
        super().__init__('multiply_server')

        self.service = self.create_service(
            AddTwoInts, 
            'multiply_two_ints',
            self.multiply_callback
        )

        self.get_logger().info("Multiply server ready")

    def multiply_callback(self, request, response):
        result = request.a * request.b
        response.sum = result

        #Custom log
        self.get_logger().info(
            f"Received request: {request.a} * {request.b} = {result}"
        )

        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = MultiplyServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()