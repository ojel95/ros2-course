#include "my_robot_interfaces/msg/hardware_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace my_robot_interfaces::msg;

class HardwareStatusPublisherNode : public rclcpp::Node
{
  public:

    HardwareStatusPublisherNode()
    : Node("hardware_status_publisher")
    {
      m_publisher = this->create_publisher<HardwareStatus>("hardware_status", 10);
      m_timer     = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&HardwareStatusPublisherNode::publishStatus, this));

      RCLCPP_INFO(this->get_logger(), "C++ hardware status publisher has started.");
    }

  private:

    void
    publishStatus()
    {
      HardwareStatus msg   = HardwareStatus();
      msg.temperature      = 40;
      msg.are_motors_ready = false;
      msg.debug_message    = "Not OK";

      m_publisher->publish(msg);
    }

  private:

    rclcpp::Publisher<HardwareStatus>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr                 m_timer;
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareStatusPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}