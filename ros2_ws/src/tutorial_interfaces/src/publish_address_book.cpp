#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

// Include the header of our newly created AddressBook.msg.
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;

// Create a node and an AddressBook publisher.
class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher()
      : Node("address_book_publisher")
  {
    address_book_publisher_ =
        this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

    // Create a callback to publish the messages periodically.
    auto publish_msg = [this]() -> void
    {
      // Create an AddressBook message instance that we will later publish.
      auto message = more_interfaces::msg::AddressBook();

      // Populate AddressBook fields.
      message.first_name = "John";
      message.last_name = "Doe";
      message.phone_number = "1234567890";
      message.phone_type = message.PHONE_TYPE_MOBILE;

      std::cout << "Publishing Contact\nFirst:" << message.first_name << "  Last:" << message.last_name << std::endl;
      // Finally send the message periodically.
      this->address_book_publisher_->publish(message);
    };
    // Create a 1 second timer to call our publish_msg function every second.
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();

  return 0;
}