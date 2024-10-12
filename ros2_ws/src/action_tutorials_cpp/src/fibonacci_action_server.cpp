// The first few lines include all of the headers we need to compile.
#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
    // Next we create a class that is a derived class of rclcpp::Node:
    class FibonacciActionServer : public rclcpp::Node
    {
    public:
        using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
        using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

        // The constructor for the FibonacciActionServer class initializes the node name as fibonacci_action_server
        explicit FibonacciActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("fibonacci_action_server", options)
        {
            using namespace std::placeholders;

            // The constructor also instantiates a new action server
            this->action_server_ = rclcpp_action::create_server<Fibonacci>(
                this,
                "fibonacci",
                std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
                std::bind(&FibonacciActionServer::handle_cancel, this, _1),
                std::bind(&FibonacciActionServer::handle_accepted, this, _1));
        }

    private:
        /*
        An action server requires 6 things:
         * The templated action type name: Fibonacci.
         * A ROS 2 node to add the action to: this.
         * The action name: 'fibonacci'.
         * A callback function for handling goals: handle_goal
         * A callback function for handling cancellation: handle_cancel.
         * A callback function for handling goal accept: handle_accept.
        */
        rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

        // The callback for handling new goals. This implementation just accepts all goals.
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const Fibonacci::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // The callback for dealing with cancellation. This implementation just tells the client that it accepted the cancellation.
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleFibonacci> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // The callbacks accepts a new goal and starts processing it. Since the execution is a long-running operation,
        // we spawn off a thread to do the actual work and return from handle_accepted quickly.
        void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
        }

        // All further processing and updates are done in the execute method in the new thread.
        // This work thread processes one sequence number of the Fibonacci sequence every second,
        // publishing a feedback update for each step. When it has finished processing, it marks the goal_handle as succeeded, and quits.
        void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(1);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Fibonacci::Feedback>();
            auto &sequence = feedback->partial_sequence;
            sequence.push_back(0);
            sequence.push_back(1);
            auto result = std::make_shared<Fibonacci::Result>();

            for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
            {
                // Check if there is a cancel request
                if (goal_handle->is_canceling())
                {
                    result->sequence = sequence;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                // Update sequence
                sequence.push_back(sequence[i] + sequence[i - 1]);
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback");

                loop_rate.sleep();
            }

            // Check if goal is done
            if (rclcpp::ok())
            {
                result->sequence = sequence;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }
    }; // class FibonacciActionServer

} // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)