#include "sketch_bot_core/robot_control.h"


namespace robot_control
{

  RobotControl::RobotControl()
    :node_(std::make_shared<rclcpp::Node>("robot_control", 
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
     move_group_interface_(node_, "ar_manipulator"),
     planning_scene_interface_(std::make_unique<moveit::planning_interface::PlanningSceneInterface>()),
     robot_model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description"))
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    thread_ = std::make_unique<std::thread>([&]() 
    {
        executor_->add_node(node_);
        executor_->spin();
        executor_->remove_node(node_);
    });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

  RobotControl::~RobotControl()
  {
    executor_->cancel();
    thread_->join();
  }

  bool RobotControl::planToJointPose(const std::vector<double>& joint_positions)
  {
    //TO DO : Assert vector size is 6
    std::vector<double> joint_positions_radians;
    for (double jt : joint_positions)
      joint_positions_radians.push_back(jt*(M_PI/180.0));

    bool within_bounds = move_group_interface_.setJointValueTarget(joint_positions_radians);
    if (!within_bounds)
    {
      RCLCPP_ERROR(node_->get_logger(), "Target joint configuration outside of limits");
      return false;
    }

    // move_group_interface_.setMaxVelocityScalingFactor(0.05);
    // move_group_interface_.setMaxAccelerationScalingFactor(0.05);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      motion_plan_ = plan;
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
      return false;
    }
  }



std::vector<std::vector<double>> RobotControl::getIKSolutions(geometry_msgs::msg::Pose goal_pose)
{

  double l4 = 0.09430;//0.085;
  double X = goal_pose.position.x;
  double Y = goal_pose.position.y;

  double r = sqrt(X * X + Y * Y) - l4;
  double theta = atan2(Y, X);

  double x = r * cos(theta);
  double y = r * sin(theta);
  double z = goal_pose.position.z;

  geometry_msgs::msg::Pose goal_pose_wrist;
  goal_pose_wrist.orientation.w = 1.0;
  goal_pose_wrist.position.x = x;
  goal_pose_wrist.position.y = y;
  goal_pose_wrist.position.z = z;
  // goal_pose_wrist.position.x = 0.148;//x;
  // goal_pose_wrist.position.y = 0.002;//y;
  // goal_pose_wrist.position.z = 0.436;//z;

  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("ar_manipulator");
  const auto& solver = joint_model_group->getSolverInstance();
  std::vector<geometry_msgs::msg::Pose> poses = { goal_pose_wrist };

  std::vector<double> dummy_seed(4, 0.0);
  std::vector<std::vector<double>> joint_results;
  kinematics::KinematicsResult result;
  kinematics::KinematicsQueryOptions options;

  if (!solver->getPositionIK(poses, dummy_seed, joint_results, result, options))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get IK solutions");
  }
  // RCLCPP_INFO_STREAM(node_->get_logger(), "6");

  for (std::size_t i = 0; i < joint_results.size(); ++i)
  {
    joint_results[i][3] = -joint_results[i][2] - joint_results[i][1] - (M_PI/2.0);

    for (std::size_t j = 0; j < joint_results[i].size(); ++j)
    {
      // if (joint_results[i][j]>M_PI && joint_results[i][j]<=M_PI*2.0)
      // {
      //   joint_results[i][j] = M_PI - joint_results[i][j];
      // }
      joint_results[i][j] = fmod((joint_results[i][j] + M_PI), (M_PI*2.0)) - M_PI;
    }
  }

    // RCLCPP_INFO_STREAM(node_->get_logger(), "==========================Joint soultions are");
    // for (size_t i = 0; i < joint_results.size(); ++i) { // Iterate through rows
    //     for (size_t j = 0; j < joint_results[i].size(); ++j) { // Iterate through elements in current row
    //         std::cout << joint_results[i][j] * (180.0/M_PI) << " ";
    //     }
    //     std::cout << std::endl; // New line after each row
    // }

  return joint_results;
}


  bool RobotControl::planToCartesianPose(geometry_msgs::msg::Pose target_pose)
  {

    // std::vector<std::vector<double>> joint_results = getIKSolutions(target_pose);



    std::vector<double> start_joints;// = {90.7102*(M_PI/180.0), -35.0407*(M_PI/180.0), 127.156*(M_PI/180.0), -2.11503*(M_PI/180.0)};

    const moveit::core::JointModelGroup* joint_model_group = 
      move_group_interface_.getCurrentState()->getJointModelGroup("ar_manipulator");
    moveit::core::RobotStatePtr current_state = move_group_interface_.getCurrentState(10);
    current_state->copyJointGroupPositions(joint_model_group, start_joints);

     std::vector<double> joints = RobotControl::getNearestIKSolution(target_pose, start_joints);





    // RCLCPP_INFO_STREAM(node_->get_logger(), "==========================Joint soultions are");
    // for (size_t i = 0; i < joint_results.size(); ++i) { // Iterate through rows
    //     for (size_t j = 0; j < joint_results[i].size(); ++j) { // Iterate through elements in current row
    //         std::cout << joint_results[i][j] * (180.0/M_PI) << " ";
    //     }
    //     std::cout << std::endl; // New line after each row
    // }

    // if (joint_results.empty())
    //   return false;


    // planToJointPose(joint_results[0]);
    // return true;

    bool within_bounds = move_group_interface_.setJointValueTarget(joints);
    if (!within_bounds)
    {
      RCLCPP_ERROR(node_->get_logger(), "Target joint configuration outside of limits");
      return false;
    }

    // move_group_interface_.setMaxVelocityScalingFactor(0.05);
    // move_group_interface_.setMaxAccelerationScalingFactor(0.05);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      motion_plan_ = plan;
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
      return false;
    }
    return true;

    // // Set a target Pose for MoveIt
    // move_group_interface_.setPoseTarget(target_pose);

    // auto const [success, plan] = [this] {
    //   moveit::planning_interface::MoveGroupInterface::Plan msg;
    //   auto const ok = static_cast<bool>(move_group_interface_.plan(msg));
    //   return std::make_pair(ok, msg);
    // }();

    // if (success)
    // {
    //   motion_plan_ = plan;
    //   return true;
    // }
    // else
    // {
    //   RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
    //   return false;
    // }
  }



double RobotControl::getCartesianDistance(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2)
{
  // Extract position components
  double x1 = p1.position.x;
  double y1 = p1.position.y;
  double z1 = p1.position.z;

  double x2 = p2.position.x;
  double y2 = p2.position.y;
  double z2 = p2.position.z;

  // Calculate the squared difference for each dimension
  double dx = x2 - x1;
  double dy = y2 - y1;
  double dz = z2 - z1;

  // Compute and return the Cartesian distance
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}



  bool RobotControl::planCartesianPath(std::vector<geometry_msgs::msg::Pose> waypoints)
  {
    // std::cout<<"Tool0 waypoints"<<std::endl;
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = move_group_interface_.getJointNames();

    std::vector<double> start_joints;// = {90.7102*(M_PI/180.0), -35.0407*(M_PI/180.0), 127.156*(M_PI/180.0), -2.11503*(M_PI/180.0)};

    const moveit::core::JointModelGroup* joint_model_group = 
      move_group_interface_.getCurrentState()->getJointModelGroup("ar_manipulator");
    moveit::core::RobotStatePtr current_state = move_group_interface_.getCurrentState(10);
    current_state->copyJointGroupPositions(joint_model_group, start_joints);

    double time_from_start = 0.0;
    double time_diff = 0.2;

    addTrajectoryPoint(trajectory, start_joints, time_from_start);
    // time_from_start += time_diff;


    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
      // std::cout<<i+1 <<" of "<<waypoints.size()<<std::endl;
      // std::cout<<waypoints[i].position.x <<", "<<waypoints[i].position.y<<", "<<waypoints[i].position.z<<std::endl;
      std::vector<double> joints = RobotControl::getNearestIKSolution(waypoints[i], start_joints);
      
      double maxDiff = 0; // Initialize with 0, or with the first difference
      for (size_t i = 0; i < joints.size(); ++i) {
          double currentDiff = std::abs(joints[i] - start_joints[i]);
          maxDiff = std::max(maxDiff, currentDiff);
      }
      time_diff = maxDiff * 10.0;

      // std::cout<<"time_diff: " <<time_diff<<std::endl;


      
      time_from_start += time_diff;
      addTrajectoryPoint(trajectory, joints, time_from_start);
      start_joints = joints;
      // time_from_start += time_diff;

    }

    // std::cout<<"Wrist waypoints"<<std::endl;
    // for (std::size_t i = 0; i < waypoints.size(); ++i)
    // {
    //   std::cout<<waypoints[i].position.x <<", "<<waypoints[i].position.y<<", "<<waypoints[i].position.z<<std::endl;
    // }
    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // moveit_msgs::msg::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    // (void)fraction;
    // for (std::size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
    // {
    //   trajectory.joint_trajectory.points[i].positions[3] = 
    //     -trajectory.joint_trajectory.points[i].positions[2] - trajectory.joint_trajectory.points[i].positions[1] + (M_PI/2.0);
    // }


    motion_plan_.trajectory_.joint_trajectory = trajectory;
    return true;
  }






void RobotControl::addTrajectoryPoint(trajectory_msgs::msg::JointTrajectory& joint_solution,
                                       const std::vector<double>& positions, double time_from_start)
{
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
  point.velocities.resize(positions.size(), 0.0);
  point.accelerations.resize(positions.size(), 0.0);
  joint_solution.points.push_back(point);
}















  void RobotControl::addCollisionMesh()
  {

  }

  void RobotControl::removeCollisionMesh()
  {

  }

  bool RobotControl::executeTrajectory()
  {
    move_group_interface_.execute(motion_plan_);
    return true;
  }

  

  bool RobotControl::getTargetPose(geometry_msgs::msg::Pose& target_pose, std::string target_frame)
  {
    geometry_msgs::msg::Pose pose;
    getPose("gripper", "link6", pose);
    geometry_msgs::msg::TransformStamped t;
    getTransform("base_link", target_frame, t);
    target_pose = applyTransform(pose, t);
    return true;
  }

  bool RobotControl::getManipulatorJointPositions(std::vector<double>& joint_positions)
  {
    const moveit::core::JointModelGroup* joint_model_group = 
      move_group_interface_.getCurrentState()->getJointModelGroup("ar_manipulator");
    moveit::core::RobotStatePtr current_state = move_group_interface_.getCurrentState(10);
    current_state->copyJointGroupPositions(joint_model_group, joint_positions);
    std::transform(joint_positions.begin(), joint_positions.end(), joint_positions.begin(), [](double d) -> double { return d * (180.0/M_PI); });
    return true;
  }

  geometry_msgs::msg::Pose RobotControl::applyRotation(const geometry_msgs::msg::Pose& pose, double roll, double pitch, double yaw)
  {
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(pose.orientation, q_orig); 
    q_rot.setRPY(roll*(M_PI/180.0), pitch*(M_PI/180.0), yaw*(M_PI/180.0));
    q_new =  q_orig*q_rot;
    q_new.normalize();
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q_new);
    geometry_msgs::msg::Pose pose_new;
    pose_new.position = pose.position;
    pose_new.orientation = msg_quat;
    return pose_new;
  }

  geometry_msgs::msg::Pose RobotControl::applyTransform(const geometry_msgs::msg::Pose& pose,
                                          const geometry_msgs::msg::TransformStamped& t)
  {
    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.pose = pose;
    pose_in.header.frame_id = t.child_frame_id;
    tf2::doTransform(pose_in, pose_out, t);
    // std::cout<<"P: "<<pose_out.pose.position.x<<" "<<pose_out.pose.position.y<<" "<<pose_out.pose.position.z<<std::endl;
    return pose_out.pose;
  }

  bool RobotControl::getTransform(std::string fromFrame, std::string toFrame, geometry_msgs::msg::TransformStamped& t)
  {
    try 
    {
      t = tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero, tf2::durationFromSec(2.0));
    } 
    catch (const tf2::TransformException & ex) 
    {
      RCLCPP_INFO(node_->get_logger(), "Could not transform %s to %s: %s",
                  toFrame.c_str(), fromFrame.c_str(), ex.what());
      return false;
    }
    // std::cout<<"T: "<<t.transform.translation.x<<" "<<t.transform.translation.y<<" "<<t.transform.translation.z<<std::endl;
    return true;
  }

  bool RobotControl::getPose(std::string fromFrame, std::string toFrame, geometry_msgs::msg::Pose& pose)
  {
    geometry_msgs::msg::TransformStamped t;
    if (getTransform(fromFrame, toFrame, t))
    {
      geometry_msgs::msg::Pose pose_zero;
      pose_zero.orientation.w = 1.0;
      pose = applyTransform(pose_zero, t);
      return true;
    }
    return false;
  }

  geometry_msgs::msg::Pose RobotControl::createROSPoseMsg(double x, double y, double z, double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::Pose pose;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll*(M_PI/180.0), pitch*(M_PI/180.0), yaw*(M_PI/180.0));
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);
    pose.orientation = msg_quat;

    return pose;
  }









std::vector<double> RobotControl::getNearestIKSolution(geometry_msgs::msg::Pose goal_pose,
                                                        std::vector<double> start_joints)
{
  // Get all collision-free IK solutions
  std::vector<std::vector<double>> joint_results = getIKSolutions(goal_pose);

  // Find the closest joint configuration to start_joints
  double min_distance = std::numeric_limits<double>::max();
  std::vector<double> nearest_solution;

  for (auto joints : joint_results)
  {
    double distance = 0.0;
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
      joints[i] = toClosestCoterminalAngle(joints[i], start_joints[i], -M_PI, M_PI);
      double diff = start_joints[i] - joints[i];
      distance += diff * diff;

    }
    distance = std::sqrt(distance);
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_solution = joints;
    }
  }

  return nearest_solution;
}

double RobotControl::toClosestCoterminalAngle(double angle, double current_angle, double lower_bound,
                                               double upper_bound)
{
  double coterminal_angle;
  std::vector<double> coterminal_angles;
  coterminal_angles.push_back(angle);

  // Get negative coterminal angles
  coterminal_angle = angle - (M_PI * 2);
  while (coterminal_angle > lower_bound)
  {
    coterminal_angles.push_back(coterminal_angle);
    coterminal_angle -= (M_PI * 2);
  }

  // Get positive coterminal angles
  coterminal_angle = angle + (M_PI * 2);
  while (coterminal_angle < upper_bound)
  {
    coterminal_angles.push_back(coterminal_angle);
    coterminal_angle += (M_PI * 2);
  }

  // Search for coterminal angle closest to the current angle
  auto it = std::min_element(coterminal_angles.begin(), coterminal_angles.end(),
                             [current_angle](double angle_a, double angle_b) {
                               return std::abs(current_angle - angle_a) < std::abs(current_angle - angle_b);
                             });

  return *it;
}







}