#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sketch_bot_core/robot_control.h>
#include <sketch_bot_core/gcode.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


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




  robot_control::RobotControl rc_;
  std::vector<double> joints = {90.0, 34.0, -127.0, -2.0};
  rc_.planToJointPose(joints);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rc_.executeTrajectory();

  // geometry_msgs::msg::Pose target_pose_approach = rc_.createROSPoseMsg(0.437, 0, 0.392, 0.0, 0.0, 0.0);
  geometry_msgs::msg::Pose target_pose_approach = rc_.createROSPoseMsg(0.242, 0.003, 0.436, 0.0, 0.0, 0.0);
  // geometry_msgs::msg::Pose target_pose_approach = rc_.createROSPoseMsg(0.4, 0.005, 0.278, 180.0, 0.0, 180.0);
  // rc_.planCartesianPath({target_pose_approach});

  rc_.planToCartesianPose(target_pose_approach);
  rclcpp::sleep_for(std::chrono::milliseconds(1500));
  rc_.executeTrajectory();

  rclcpp::sleep_for(std::chrono::milliseconds(1500));



  gcode::Gcode gcode;
  gcode.setOrigin(0.242, 0.003, 0.436);
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("sketch_bot_core");
  std::string gcode_file_path = package_share_directory + "/gcode/apple.gcode";
  geometry_msgs::msg::PoseArray pose_array = gcode.toPoseArray(gcode_file_path);
  rc_.planCartesianPath(pose_array.poses);
  rclcpp::sleep_for(std::chrono::milliseconds(1500));
  rc_.executeTrajectory();

  // Shutdown ROS
  // rclcpp::shutdown();
  spinner.join();
  return 0;
}
