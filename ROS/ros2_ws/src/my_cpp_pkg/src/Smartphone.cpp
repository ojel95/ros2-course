#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class SmartphoneNode : public rclcpp::Node
{
  public:

    SmartphoneNode()
    : Node("smartphone")
    {
      m_subscriber = this->create_subscription<example_interfaces::msg::String>(
        "robot_news",
        10,
        std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
    }

  private:

    void
    callbackRobotNews(example_interfaces::msg::String::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

  private:

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr m_subscriber;
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmartphoneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}