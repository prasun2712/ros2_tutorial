#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/*
The subscriber node’s code is nearly identical to the publisher’s. Now the node is named minimal_subscriber,
and the constructor uses the node’s create_subscription class to execute the callback.

There is no timer because the subscriber simply responds whenever data is published to the topic topic.
*/
class MinimalSubscriber : public rclcpp::Node
{
public:
    // There is no timer because the subscriber simply responds whenever data is published to the topic topic.
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    /*
    Recall from the topic tutorial that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.

    The topic_callback function receives the string message data published over the topic, and simply writes it to the console using the RCLCPP_INFO macro.

    The only field declaration in this class is the subscription.
    */
    // The topic_callback function receives the string message data published over the topic.
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    /*
    The main function is exactly the same, except now it spins the MinimalSubscriber node. For the publisher node, spinning meant starting the timer,
    but for the subscriber it simply means preparing to receive messages whenever they come.
    */
    rclcpp::init(argc, argv);
    // The main function is exactly the same, except now it spins the MinimalSubscriber node.
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}