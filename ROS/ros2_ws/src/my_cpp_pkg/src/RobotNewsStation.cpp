#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <example_interfaces/msg/detail/string__struct.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>

class RobotNewsStationNode : public rclcpp::Node
{
  public:

    RobotNewsStationNode()
    : Node("robot_news_station")
    , m_robotName("CppRobot")
    {
      m_publisher = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
      m_timer     = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RobotNewsStationNode::publishNews, this));

      RCLCPP_INFO(this->get_logger(), "Robot News Station has been started.");
    }

  private:

    void
    publishNews()
    {
      auto msg = example_interfaces::msg::String();

      msg.data = std::string("Hi, this is ") + m_robotName + std::string("from the Robot News Station");
      m_publisher->publish(msg);
    }

  private:

    std::string                                                   m_robotName;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr                                  m_timer;
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNewsStationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}