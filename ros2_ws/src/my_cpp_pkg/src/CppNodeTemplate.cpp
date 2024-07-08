#include "rclcpp/rclcpp.hpp"

class NumberCounterNode : public rclcpp::Node // MODIFY NAME
{
  public:

    NumberCounterNode()
    : Node("node_name") // MODIFY NAME
    {
    }

  private:
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}