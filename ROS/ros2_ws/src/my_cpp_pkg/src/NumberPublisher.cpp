#include "example_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

class NumberPublisherNode : public rclcpp::Node
{
  public:

    NumberPublisherNode()
    : Node("number_publisher")
    , m_number(2)
    {
      m_publisher = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
      m_timmer    = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&NumberPublisherNode::publishNumber, this));

      RCLCPP_INFO(this->get_logger(), "C++ Number publisher has started.");
    }

  private:

    void
    publishNumber()
    {
      auto msg = example_interfaces::msg::Int64();

      msg.data = m_number;

      m_publisher->publish(msg);
    }

  private:

    long                                                         m_number;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr                                 m_timmer;
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}