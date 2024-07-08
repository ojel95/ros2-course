#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include <example_interfaces/srv/detail/set_bool__struct.hpp>
#include <functional>
#include <rclcpp/logging.hpp>

using namespace example_interfaces;

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
  public:

    NumberCounterNode()
    : Node("number_counter")
    , m_count(0)
    {
      m_subscriber = this->create_subscription<msg::Int64>(
        "number",
        10,
        std::bind(&NumberCounterNode::onNumberReceived, this, std::placeholders::_1));

      m_publisher = this->create_publisher<msg::Int64>("number_count", 10);

      m_server = this->create_service<srv::SetBool>(
        "reset_counter",
        std::bind(&NumberCounterNode::onResetCounter, this, _1, _2));

      RCLCPP_INFO(this->get_logger(), "C++ number counter has started.");
    }

  private:

    void
    publishCounter()
    {
      auto msg = msg::Int64();

      msg.data = m_count;

      m_publisher->publish(msg);
    }

    void
    onNumberReceived(msg::Int64 msg)
    {
      m_count += msg.data;

      RCLCPP_INFO(this->get_logger(), "%ld", m_count);
      publishCounter();
    }

    void
    onResetCounter(srv::SetBool::Request::SharedPtr request, srv::SetBool::Response::SharedPtr response)
    {
      if (request->data == true)
      {
        const std::string message = "Reseting counter at cpp number counter.";
        RCLCPP_INFO(this->get_logger(), message.c_str());
        m_count           = 0;
        response->success = true;
        response->message = message;
      }
      else
      {
        const std::string message = "Received false reset request. Counter will not be reseted.";
        RCLCPP_WARN(this->get_logger(), message.c_str());
        response->success = false;
        response->message = message;
      }
    }

  private:

    rclcpp::Subscription<msg::Int64>::SharedPtr m_subscriber;
    rclcpp::Publisher<msg::Int64>::SharedPtr    m_publisher;
    rclcpp::Service<srv::SetBool>::SharedPtr    m_server;
    long                                        m_count;
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