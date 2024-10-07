#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/*
The next piece of code creates the class and the constructor. The first line of this constructor creates a parameter with the name my_parameter 
and a default value of world. The parameter type is inferred from the default value, so in this case it would be set to a string type.
Next the timer_ is initialized with a period of 1000ms, which causes the timer_callback function to be executed once a second.
*/
class MinimalParam : public rclcpp::Node
{
public:
    MinimalParam()
        : Node("minimal_param_node")
    {
        // Optionally, you can set a descriptor for the parameter. Descriptors allow you to specify a text description of the parameter and its constraints, 
        // like making it read-only, specifying a range, etc. For that to work, the code in the constructor has to be changed to:
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter is my custom param containing 'world' as a default value!";
        this->declare_parameter("my_parameter", "world", param_desc);
        // The rest of the code remains the same. Once you run the node, 
        // you can then run "ros2 param describe /minimal_param_node my_parameter" to see the type and description.

        timer_ = this->create_wall_timer(
            1000ms, std::bind(&MinimalParam::timer_callback, this));
    }

    /*
    The first line of our timer_callback function gets the parameter my_parameter from the node, 
    and stores it in my_param. Next the RCLCPP_INFO function ensures the event is logged. 
    The set_parameters function then sets the parameter my_parameter back to the default string value world.
    In the case that the user changed the parameter externally, this ensures it is always reset back to the original.
    */
    void timer_callback()
    {
        std::string my_param = this->get_parameter("my_parameter").as_string();

        RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

        std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
        this->set_parameters(all_new_parameters);
    }

private:
    // Last is the declaration of timer_.
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    /*
    Following our MinimalParam is our main. Here ROS 2 is initialized, an instance of the MinimalParam class is constructed, 
    and rclcpp::spin starts processing data from the node.
    */
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalParam>());
    rclcpp::shutdown();
    return 0;
}