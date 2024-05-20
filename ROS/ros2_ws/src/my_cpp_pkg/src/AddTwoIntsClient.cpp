#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace example_interfaces::srv;

class AddTwoIntsClientNode : public rclcpp::Node
{
  public:

    AddTwoIntsClientNode()
    : Node("add_two_ints_client")
    {
      m_threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 2, 3)));
      m_threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 4, 5)));
    }

    void
    callAddTwoIntsService(const int a, const int b)
    {
      auto client = this->create_client<AddTwoInts>("add_two_ints");

      while (!client->wait_for_service(std::chrono::seconds(1)))
        RCLCPP_WARN(this->get_logger(), "Waiting for service add_two_ints.");

      auto request = std::make_shared<AddTwoInts::Request>();
      request->a   = a;
      request->b   = b;

      auto future = client->async_send_request(request);

      try
      {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "%d + %d = %ld", a, b, response->sum);
      }
      catch (const std::exception& error)
      {
        RCLCPP_ERROR(this->get_logger(), "Error calling the service");
      }
    }

  private:

    std::vector<std::thread> m_threads;
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}