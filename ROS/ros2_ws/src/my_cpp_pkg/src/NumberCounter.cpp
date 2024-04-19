#include "example_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include <example_interfaces/msg/detail/int64__struct.hpp>
#include <rclcpp/publisher.hpp>

class NumberCounterNode : public rclcpp::Node
{
  public:

    NumberCounterNode()
    : Node("number_counter")
    , m_count(0)
    {
      m_subscriber = this->create_subscription<example_interfaces::msg::Int64>(
        "number",
        10,
        std::bind(&NumberCounterNode::onNumberReceived, this, std::placeholders::_1));

      m_publisher = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

      RCLCPP_INFO(this->get_logger(), "C++ number counter has started.");
    }

  private:

    void
    publishCounter()
    {
      auto msg = example_interfaces::msg::Int64();

      msg.data = m_count;

      m_publisher->publish(msg);
    }

    void
    onNumberReceived(example_interfaces::msg::Int64 msg)
    {
      RCLCPP_INFO(this->get_logger(), "%ld", msg.data);
      m_count += msg.data;

      publishCounter();
    }

  private:

    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr m_subscriber;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr    m_publisher;
    long                                                            m_count;
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}