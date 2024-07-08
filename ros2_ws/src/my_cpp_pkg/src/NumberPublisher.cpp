#include "example_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

class NumberPublisherNode : public rclcpp::Node
{
  public:

    NumberPublisherNode()
    : Node("number_publisher")
    {
      this->declare_parameter("number_to_publish", 2);
      this->declare_parameter("publish_frequency", 1.0);

      m_number                      = this->get_parameter("number_to_publish").as_int();
      const double publishFrequency = this->get_parameter("publish_frequency").as_double();

      m_publisher = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
      m_timer     = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000.0 / publishFrequency)),
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
    rclcpp::TimerBase::SharedPtr                                 m_timer;
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