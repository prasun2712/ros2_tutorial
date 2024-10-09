# The only different import statement for the client is import sys. The client node code uses sys.argv to get access to command line input arguments for the request.
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    # The constructor definition creates a client with the same type and name as the service node. 
    # The type and name must match for the client and service to be able to communicate.
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # The while loop in the constructor checks if a service matching the type and name of the client is available once a second.
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    # Below the constructor is the request definition, followed by main.
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

# The only significant difference in the client’s main is the while loop. 
# The loop checks the future to see if there is a response from the service, as long as the system is running. 
# If the service has sent a response, the result will be written in a log message.
def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()