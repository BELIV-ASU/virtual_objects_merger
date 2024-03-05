#include "virtual_objects_merger/virtual_objects_merger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<VirtualObjectsMerger>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}