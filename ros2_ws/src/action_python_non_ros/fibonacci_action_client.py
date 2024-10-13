#!/usr/bin/env python3.10.12
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci

# A class FibonacciActionClient that is a subclass of Node.
class FibonacciActionClient(Node):

    def __init__(self):
        # The class is initialized by calling the Node constructor, naming our node fibonacci_action_client.
        super().__init__('fibonacci_action_client')

        # Also in the class constructor, we create an action client using the custom action definition.
        # We create an ActionClient by passing it three arguments:
        #  - A ROS 2 node to add the action client to: self
        #  - The type of the action: Fibonacci
        #  - The action name: 'fibonacci'
        # Our action client will be able to communicate with action servers of the same action name and type.
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    # Define a method send_goal in the FibonacciActionClient class.
    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        # The ActionClient.send_goal_async() method returns a future to a goal handle. First we register a callback for when the future is complete.
        # We need to register the callback with the action client. This is achieved by additionally passing the callback to the action client when we send a goal.
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Note that the future is completed when an action server accepts or rejects the goal request. 
    # Let’s look at the goal_response_callback in more detail. We can check to see if the goal was rejected and return early since we know there will be no result.
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Now that we’ve got a goal handle, we can use it to request the result with the method get_result_async(). 
        # Similar to sending the goal, we will get a future that will complete when the result is ready. 
        # Let’s register a callback just like we did for the goal response.
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # In the callback, we log the result sequence and shutdown ROS 2 for a clean exit.
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    # Here’s the callback function for feedback messages.
    # In the callback we get the feedback portion of the message and print the partial_sequence field to the screen.
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()