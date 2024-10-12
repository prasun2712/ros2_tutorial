// The first few lines include all of the headers we need to compile.
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
    // Next we create a class that is a derived class of rclcpp::Node
    class FibonacciActionClient : public rclcpp::Node
    {
    public:
        using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
        using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

        // The constructor for the FibonacciActionClient class initializes the node name as fibonacci_action_client
        explicit FibonacciActionClient(const rclcpp::NodeOptions &options)
            : Node("fibonacci_action_client", options)
        {
            /*
            The constructor also instantiates a new action client

            An action client requires 3 things:
             * The templated action type name: Fibonacci.
             * A ROS 2 node to add the action client to: this.
             * The action name: 'fibonacci'.
            */
            this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
                this,
                "fibonacci");

            // Instantiate a ROS timer that will kick off the one and only call to send_goal. When the timer expires, it will call send_goal.
            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&FibonacciActionClient::send_goal, this));
        }

        /*
        This function does the following:
         * Cancels the timer (so it is only called once).
         * Waits for the action server to come up.
         * Instantiates a new Fibonacci::Goal.
         * Sets the response, feedback, and result callbacks.
         * Sends the goal to the server.
        */
        void send_goal()
        {
            using namespace std::placeholders;

            this->timer_->cancel();

            if (!this->client_ptr_->wait_for_action_server())
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = Fibonacci::Goal();
            goal_msg.order = 10;

            RCLCPP_INFO(this->get_logger(), "Sending goal");

            auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
            send_goal_options.feedback_callback =
                std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
            send_goal_options.result_callback =
                std::bind(&FibonacciActionClient::result_callback, this, _1);
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

    private:
        rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;

        // When the server receives and accepts the goal, it will send a response to the client. That response is handled by goal_response_callback.
        void goal_response_callback(const GoalHandleFibonacci::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        // Assuming the goal was accepted by the server, it will start processing. Any feedback to the client will be handled by the feedback_callback.
        void feedback_callback(
            GoalHandleFibonacci::SharedPtr,
            const std::shared_ptr<const Fibonacci::Feedback> feedback)
        {
            std::stringstream ss;
            ss << "Next number in sequence received: ";
            for (auto number : feedback->partial_sequence)
            {
                ss << number << " ";
            }
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        }

        // When the server is finished processing, it will return a result to the client. The result is handled by the result_callback.
        void result_callback(const GoalHandleFibonacci::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
            }
            std::stringstream ss;
            ss << "Result received: ";
            for (auto number : result.result->sequence)
            {
                ss << number << " ";
            }
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            rclcpp::shutdown();
        }
    }; // class FibonacciActionClient

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)