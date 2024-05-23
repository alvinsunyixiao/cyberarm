#include <rclcpp/rclcpp.hpp>

class CybergearNode : public rclcpp::Node {

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CybergearNode>());
  rclcpp::shutdown();
}
