#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstddef>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>

class MyNode : public rclcpp::Node
{
  public:

    MyNode()
    : Node("cpp_test")
    , m_counter(0)
    {
      RCLCPP_INFO(this->get_logger(), "Hello from cpp class");

      m_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timerCallback, this));
    }

  private:

    void
    timerCallback()
    {
      ++m_counter;
      RCLCPP_INFO(this->get_logger(), "Hello from callback %d", m_counter);
    }

  private:

    rclcpp::TimerBase::SharedPtr m_timer;
    int                          m_counter;
};

int
main(int argc, char** argv)
{
  // Initialize ROS2 communications.
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}