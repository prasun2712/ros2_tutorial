# The first import statement imports the AddTwoInts service type from the example_interfaces package. 
# The following import statement imports the ROS 2 Python client library, and specifically the Node class.

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    # The MinimalService class constructor initializes the node with the name minimal_service. Then, it creates a service and defines the type, name, and callback.
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    # The definition of the service callback receives the request data, sums it, and returns the sum as a response.
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

# Finally, the main class initializes the ROS 2 Python client library, 
# instantiates the MinimalService class to create the service node and spins the node to handle callbacks.
# To call the service from command line :
# ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 0}"
def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()