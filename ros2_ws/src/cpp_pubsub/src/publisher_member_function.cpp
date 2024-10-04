#include <chrono>
#include <functional>
#include <memory>
#include <string>

/*
Nodes are executable processes that communicate over the ROS graph. In this tutorial,
the nodes will pass information in the form of string messages to each other over a topic.
The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.
*/

/*
The top of the code includes the standard C++ headers you will be using. After the standard C++ headers is the rclcpp/rclcpp.hpp
include which allows you to use the most common pieces of the ROS 2 system. Last is std_msgs/msg/string.hpp,
which includes the built-in message type you will use to publish data.
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// The node class MinimalPublisher is created by inheriting from rclcpp::Node
/*
The next line creates the node class MinimalPublisher by inheriting from rclcpp::Node. Every this in the code is referring to the node.
*/
class MinimalPublisher : public rclcpp::Node
{
public:
    /*
    The public constructor names the node minimal_publisher and initializes count_ to 0.
    Inside the constructor, the publisher is initialized with the String message type, the topic name topic,
    and the required queue size to limit messages in the event of a backup. Next, timer_ is initialized,
    which causes the timer_callback function to be executed twice a second.
    */
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    // The timer_callback function is where the message data is set and the messages are actually published.
    /*
    The timer_callback function is where the message data is set and the messages are actually published.
    The RCLCPP_INFO macro ensures every published message is printed to the console.
    */
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    /*
    Last is the declaration of the timer, publisher, and counter fields.
    */
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    /*
    Following the MinimalPublisher class is main, where the node actually executes. rclcpp::init initializes ROS 2,
    and rclcpp::spin starts processing data from the node, including callbacks from the timer.
    */
    rclcpp::init(argc, argv);
    // rclcpp::spin starts processing data from the node, including callbacks from the timer.
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}