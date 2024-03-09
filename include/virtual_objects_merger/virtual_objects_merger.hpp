#ifndef VIRTUAL_OBJECTS_MERGER__VIRTUAL_OBJECTS_MERGER_NODE_HPP_
#define VIRTUAL_OBJECTS_MERGER__VIRTUAL_OBJECTS_MERGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"

// TF2
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;

class VirtualObjectsMerger : public rclcpp::Node
{
public:
  explicit VirtualObjectsMerger();

private:
  // Subscriber
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_{};
  std::vector<rclcpp::Subscription<DetectedObjects>::SharedPtr> sub_objects_array{};

  // Callback
  void onData(const DetectedObjects::ConstSharedPtr msg, size_t array_number);

  // Data Buffer
  std::vector<DetectedObjects::ConstSharedPtr> objects_data_{};
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_;

  // Publisher
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_{};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};
  void onTimer();
  void setupTF();

  // Parameter
  std::vector<std::string> topic_names{};
  double update_rate_hz{};
  std::string new_frame_id{};

  // Function
  autoware_auto_perception_msgs::msg::DetectedObjects transformObjects(
    autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr,
    const std::string&  
  );
  
  // Core
  size_t input_topic_size;
};

#endif  // VIRTUAL_OBJECTS_MERGER__VIRTUAL_OBJECTS_MERGER_NODE_HPP_
