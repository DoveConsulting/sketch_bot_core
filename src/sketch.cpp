#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sketch_bot_core/robot_control.h>
#include <sketch_bot_core/gcode.h>


int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "sketch", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("sketch");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });


  gcode::Gcode gcode;

  robot_control::RobotControl rc_;
  std::vector<double> joints = {0.0, 30.0, 45.0, 104.0};
  // rc_.planToJointPose(joints);
  // rclcpp::sleep_for(std::chrono::milliseconds(100));
  // rc_.executeTrajectory();

  // geometry_msgs::msg::Pose target_pose_approach = rc_.createROSPoseMsg(0.437, 0, 0.392, 0.0, 0.0, 0.0);
  geometry_msgs::msg::Pose target_pose_approach = rc_.createROSPoseMsg(0.242, 0.003, 0.436, 0.0, 0.0, 0.0);
  // geometry_msgs::msg::Pose target_pose_approach = rc_.createROSPoseMsg(0.4, 0.005, 0.278, 180.0, 0.0, 180.0);
  // rc_.planCartesianPath({target_pose_approach});

  rc_.planToCartesianPose(target_pose_approach);
  rclcpp::sleep_for(std::chrono::milliseconds(1500));
  rc_.executeTrajectory();

  // Shutdown ROS
  // rclcpp::shutdown();
  spinner.join();
  return 0;
}
