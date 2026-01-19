import rclpy
from rclpy.node import Node

from joke_teller_interfaces.srv import MultiplyTwoInts

class MultiplyClient(Node):

    def __init__(self):
        super().__init__('multiply_client')

        self.client = self.create_client(
            MultiplyTwoInts,
            'multiply_two_ints'
        )

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for multiply service...')
        
        self.request = MultiplyTwoInts.Request()

    def send_request(self, a, b):
        self.request.a=a
        self.request.b=b

        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self,future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Result: {self.request.a} * {self.request.b} = {response.product}"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        #Shutdown after recieving response
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MultiplyClient()
    node.send_request(6, 7)
    rclpy.spin(node)

if __name__ == 'main':
    main()