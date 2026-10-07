#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mcl/mcl_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mcl::MclNode>());
  rclcpp::shutdown();
  return 0;
}
