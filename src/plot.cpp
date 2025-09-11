#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class ToolTrailPublisher : public rclcpp::Node
{
public:
  ToolTrailPublisher() : Node("plot"), marker_id_(0)
  {
    double publish_rate = 10;


    // Initialize TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize marker array publisher
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("sketch_plot", 10);

    // Set up a timer to continuously look up transforms and publish markers
    auto publish_period = 1s / publish_rate;
    timer_ = this->create_wall_timer(publish_period, std::bind(&ToolTrailPublisher::publish_trail, this));

    RCLCPP_INFO(this->get_logger(), "Publishing tool trail for %s -> %s", tool_frame_.c_str(), base_frame_.c_str());
  }

private:
  void publish_trail()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // Look up the transform from the tool frame to the base frame
      transform_stamped = tf_buffer_->lookupTransform(base_frame_, tool_frame_, tf2::TimePointZero, 50ms);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not transform %s to %s: %s",
        tool_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    if (transform_stamped.transform.translation.z>0.362)
    {
      return;
    }

    // Create a new marker for the current pose
    visualization_msgs::msg::Marker new_marker;
    new_marker.header.frame_id = base_frame_;
    new_marker.header.stamp = this->get_clock()->now();
    new_marker.ns = "plot";
    new_marker.id = marker_id_++;
    new_marker.type = visualization_msgs::msg::Marker::SPHERE;
    new_marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose of the marker from the transform
    new_marker.pose.position.x = transform_stamped.transform.translation.x;
    new_marker.pose.position.y = transform_stamped.transform.translation.y;
    new_marker.pose.position.z = transform_stamped.transform.translation.z;
    new_marker.pose.orientation = transform_stamped.transform.rotation;

    RCLCPP_INFO_STREAM(this->get_logger(), "Z: "<<new_marker.pose.position.z);

    // Set marker scale and color
    new_marker.scale.x = 0.01;
    new_marker.scale.y = 0.01;
    new_marker.scale.z = 0.01;
    new_marker.color.a = 1.0;
    new_marker.color.r = 1.0;
    new_marker.color.g = 0.0;
    new_marker.color.b = 0.0;

    // Add the new marker to the array
    marker_array_.markers.push_back(new_marker);

    // Publish the entire marker array
    marker_publisher_->publish(marker_array_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  visualization_msgs::msg::MarkerArray marker_array_;
  std::string base_frame_ = "base_link";
  std::string tool_frame_ = "tip";
  int marker_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolTrailPublisher>());
  rclcpp::shutdown();
  return 0;
}
