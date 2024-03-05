#include "virtual_objects_merger/virtual_objects_merger.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

using namespace std::literals;

VirtualObjectsMerger::VirtualObjectsMerger()
: Node("virtual_objects_merger")
{

  declare_parameter("input_topics", std::vector<std::string>());
	update_rate_hz = declare_parameter<double>("update_rate_hz", 20.0);
  new_frame_id = declare_parameter<std::string>("new_frame_id", "base_link");

  topic_names = get_parameter("input_topics").as_string_array();
  if (topic_names.empty()) {
    RCLCPP_ERROR(get_logger(), "Need a 'input_topics' parameter to be set before continuing");
    return;
  }

  input_topic_size = topic_names.size();
  if (input_topic_size == 1) {
    RCLCPP_ERROR(get_logger(), "Only one topic given. Need at least two topics to continue.");
    return;
  }

  // Subscriber
  sub_objects_array.resize(input_topic_size);

  for (size_t i = 0; i < input_topic_size; i++) {
    std::function<void(const DetectedObjects::ConstSharedPtr msg)> func =
      std::bind(&VirtualObjectsMerger::onData, this, std::placeholders::_1, i);
    sub_objects_array.at(i) =
      create_subscription<DetectedObjects>(topic_names.at(i), rclcpp::QoS{1}, func);
  }

  // Publisher
  pub_objects_ = create_publisher<DetectedObjects>("~/output/objects", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&VirtualObjectsMerger::onTimer, this));

	// Setting TF
	setupTF();
}

void VirtualObjectsMerger::setupTF()
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

autoware_auto_perception_msgs::msg::DetectedObjects VirtualObjectsMerger::transformObjects(
													autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr objects,
													const std::string& target_frame_id
)
{
	autoware_auto_perception_msgs::msg::DetectedObjects output_objects = *objects;

	if (objects->header.frame_id == target_frame_id) 
	{
    return output_objects;
  }

	// Transforms the pose between the source frame and target frame
	geometry_msgs::msg::PoseStamped pose_in_;
	geometry_msgs::msg::PoseStamped pose_out_;
	pose_in_.header = objects->header;

	output_objects.header.frame_id = target_frame_id;
  for (auto& object : output_objects.objects) 
	{
  	pose_in_.pose = object.kinematics.pose_with_covariance.pose;
    tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(pose_in_, pose_out_, target_frame_id,
        tf2::Duration(std::chrono::seconds(1)));
		object.kinematics.pose_with_covariance.pose = pose_out_.pose;		
  }

	return output_objects;
}

void VirtualObjectsMerger::onData(DetectedObjects::ConstSharedPtr msg, const size_t array_number)
{
  objects_data_.push_back(msg);
}

void VirtualObjectsMerger::onTimer()
{

  if (objects_data_.size() == 0)
  {
    return;
  }

	DetectedObjects output_objects;
  //output_objects.header.stamp = rclcpp::Clock().now();
  output_objects.header.frame_id = new_frame_id;

  for (int i = 0; i < objects_data_.size(); i++) 
	{

    DetectedObjects transformed_objects =
      transformObjects(objects_data_.at(i), new_frame_id);

    output_objects.objects.insert(
      output_objects.objects.end(), std::begin(transformed_objects.objects),
      std::end(transformed_objects.objects));

		//Set time	
		output_objects.header = objects_data_.at(i)->header;

	}
  
  pub_objects_->publish(output_objects);
  objects_data_.clear();

}