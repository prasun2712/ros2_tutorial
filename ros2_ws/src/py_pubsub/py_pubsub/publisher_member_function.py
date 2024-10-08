# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The first lines of code after the comments import rclpy so its Node class can be used.
import rclpy
from rclpy.node import Node

# The next statement imports the built-in string message type that the node uses to structure the data that it passes on the topic.
from std_msgs.msg import String

# Next, the MinimalPublisher class is created, which inherits from (or is a subclass of) Node.
class MinimalPublisher(Node):

    # Following is the definition of the class’s constructor. super().__init__ calls the Node class’s constructor and gives it your node name, 
    # in this case minimal_publisher. 
    # create_publisher declares that the node publishes messages of type String (imported from the std_msgs.msg module), 
    # over a topic named topic, and that the “queue size” is 10. 
    # Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.
    # Next, a timer is created with a callback to execute every 0.5 seconds. self.i is a counter used in the callback.
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # timer_callback creates a message with the counter value appended, and publishes it to the console with get_logger().info.
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    # First the rclpy library is initialized, then the node is created, and then it “spins” the node so its callbacks are called.
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
