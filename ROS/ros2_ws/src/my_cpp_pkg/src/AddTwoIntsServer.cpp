#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace example_interfaces::srv;
using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServerNode : public rclcpp::Node
{
  public:

    AddTwoIntsServerNode()
    : Node("add_two_ints_server")
    {
      m_server = this->create_service<AddTwoInts>(
        "add_two_ints",
        std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));

      RCLCPP_INFO(this->get_logger(), "Cpp add_two_ints_server has started.");
    }

  private:

    void
    callbackAddTwoInts(
      const AddTwoInts::Request::SharedPtr  request,
      const AddTwoInts::Response::SharedPtr response)
    {
      response->sum = request->a + request->b;
      RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
    }

  private:

    rclcpp::Service<AddTwoInts>::SharedPtr m_server;
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}