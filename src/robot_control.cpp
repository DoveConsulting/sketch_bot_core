#include "sketch_bot_core/robot_control.h"


namespace robot_control
{

  RobotControl::RobotControl()
    :node_(std::make_shared<rclcpp::Node>("robot_control", 
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
     move_group_interface_(node_, "ar_manipulator"),
     // move_group_interface_gripper_(node_, "robotiq_gripper"),
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

    // gz_link_attacher_client_ = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    // gz_link_detacher_client_ = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
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





// std::vector<std::vector<double>> RobotControl::getIKSolutions(geometry_msgs::msg::Pose goal_pose)
// {
//   RCLCPP_INFO_STREAM(node_->get_logger(), "1");
//   const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
//   RCLCPP_INFO_STREAM(node_->get_logger(), "2");
//   const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("ar_manipulator");
//   RCLCPP_INFO_STREAM(node_->get_logger(), "3");
//   const auto& solver = joint_model_group->getSolverInstance();
//   RCLCPP_INFO_STREAM(node_->get_logger(), "4");
//   // std::vector<std::string> joint_names = solver->getJointNames();
//   RCLCPP_INFO_STREAM(node_->get_logger(), "5");
//   std::vector<geometry_msgs::msg::Pose> poses = { goal_pose };

//   std::vector<double> dummy_seed(4, 0.0);
//   std::vector<std::vector<double>> joint_results;
//   kinematics::KinematicsResult result;
//   kinematics::KinematicsQueryOptions options;

//   if (!solver->getPositionIK(poses, dummy_seed, joint_results, result, options))
//   {
//     RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get IK solutions");
//   }
//   RCLCPP_INFO_STREAM(node_->get_logger(), "6");
//   return joint_results;
// }




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
  goal_pose_wrist.position.x = x;
  goal_pose_wrist.position.y = y;
  goal_pose_wrist.position.z = z;


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
  RCLCPP_INFO_STREAM(node_->get_logger(), "6");

  for (std::size_t i = 0; i < joint_results.size(); ++i)
  {
    joint_results[i][3] = -joint_results[i][2] - joint_results[i][1] + (M_PI/2.0);
  }


  return joint_results;



















  // std::vector<std::vector<double>> joint_results;

  // double l1 = 0.036;
  // double L = 0.356;
  // double M = 0.352;
  // double N = 0.085;

  // double x = goal_pose.position.x;
  // double y = goal_pose.position.y;
  // double z = goal_pose.position.z - l1;

  // double phi = M_PI;//M_PI;//M_PI; 

  // double R = sqrt(x * x + y * y);

  // double s = R - N;
  // double Q = sqrt(s * s + z * z);

  // double f = atan2(z, s);

  // double g = acos(((L*L) + (Q*Q) - (M*M)) / (2 * L * Q));

  // double theta2 = f + g - M_PI/2;

  // double theta3 =  acos(((M*M) + (L*L) - (Q*Q)) / (2 * L * M));

  // double theta4 = -theta3 -theta2 + phi;

  // double theta1 = atan2(y, x);

  // joint_results.push_back({theta1, theta2, theta3, theta4});


  // std::vector<std::vector<double>> joint_results;

  // double l1 = 0.036;
  // double l2 = 0.356;
  // double l3 = 0.352;
  // double l4 = 0.085;

  // double x = goal_pose.position.x;
  // double y = goal_pose.position.y;
  // double z = goal_pose.position.z - l1;

  // double phi = M_PI;//M_PI; 



  //   // // 1. Calculate base rotation (theta1)
  //   double theta1 = atan2(y, x);

  //   // double A = x - (l4 * cos(theta1) * cos(phi)); 
  //   // double B = y - (l4 * sin(theta1) * cos(phi));  
  //   // double C = z - (l4 * sin(phi)) - l1; 

  //   // double theta3 = acos(((A*A) + (B*B) + (C*C) - (l2*l2) - (l3*l3)) / (2 * l2 * l3));








  //   // 2. Calculate wrist position
  //   double R = sqrt(x * x + y * y);

  //   // 3. Solve for shoulder (theta2) and elbow (theta3)
  //   double D = sqrt(R * R + z * z);

  //   // Check if the target is reachable
  //   // if (D > l1 + l2 || D < abs(l1 - l2)) {
  //   //     std::cout << "Target is unreachable." << std::endl;
  //   //     return {{0, 0, 0, 0}}; // Return invalid angles
  //   // }

  //   // Solve for theta3 using Law of Cosines
  //   // The elbow-up solution is shown here.
  //   double theta3 = acos((l1 * l1 + l2 * l2 - D * D) / (2 * l1 * l2));













  //   double a = l3 * sin(theta3);
  //   double b = l2 + (l3 * cos(theta3));
  //   double c = z - l1 - (l4*sin(phi));
  //   double r = sqrt((a*a) + (b*b));

  //   double theta2 = atan2(c, sqrt((r*r) - (c*c))) - atan2(a, b);

  //   double theta4 = phi - theta2 - theta3;

  //   joint_results.push_back({theta1, theta2, theta3, theta4});

    // // 2. Calculate wrist position
    // double R = sqrt(x * x + y * y);

    // // 3. Solve for shoulder (theta2) and elbow (theta3)
    // double D = sqrt(R * R + z * z);

    // // Check if the target is reachable
    // if (D > L1 + L2 || D < abs(L1 - L2)) {
    //     std::cout << "Target is unreachable." << std::endl;
    //     return {NAN, NAN, NAN, NAN}; // Return invalid angles
    // }

    // // Solve for theta3 using Law of Cosines
    // // The elbow-up solution is shown here.
    // angles.theta3 = acos((L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2));

    // // Solve for theta2 using Law of Cosines
    // double gamma = atan2(z, R);
    // double beta = acos((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D));
    // // The elbow-up solution
    // angles.theta2 = gamma + beta;

    // // 4. Set the wrist rotation (theta4)
    // angles.theta4 = wrist_rot;





  return joint_results;
}










  bool RobotControl::planToCartesianPose(geometry_msgs::msg::Pose target_pose)
  {

    std::vector<std::vector<double>> joint_results = getIKSolutions(target_pose);

    RCLCPP_INFO_STREAM(node_->get_logger(), "==========================Joint soultions are");
    for (size_t i = 0; i < joint_results.size(); ++i) { // Iterate through rows
        for (size_t j = 0; j < joint_results[i].size(); ++j) { // Iterate through elements in current row
            std::cout << joint_results[i][j] * (180.0/M_PI) << " ";
        }
        std::cout << std::endl; // New line after each row
    }

    if (joint_results.empty())
      return false;


    // planToJointPose(joint_results[0]);
    // return true;

    bool within_bounds = move_group_interface_.setJointValueTarget(joint_results[0]);
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

  bool RobotControl::planCartesianPath(std::vector<geometry_msgs::msg::Pose> waypoints)
  {
    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    (void)fraction;
    motion_plan_.trajectory_ = trajectory;
    return true;
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

  // bool RobotControl::setGripperPosition(double joint_position)
  // {
  //   std::vector<double> joint_directions = {1.0, 1.0, -1.0, -1.0, -1.0, 1.0};
  //   std::vector<double> joint_positions_radians;
  //   for (double direction : joint_directions)
  //     joint_positions_radians.push_back((direction*joint_position)*(M_PI/180.0));

  //   bool within_bounds = move_group_interface_gripper_.setJointValueTarget(joint_positions_radians);
  //   if (!within_bounds)
  //   {
  //     RCLCPP_ERROR(node_->get_logger(), "Target joint configuration for gripper outside of limits");
  //     return false;
  //   }

  //   move_group_interface_gripper_.setMaxVelocityScalingFactor(0.05);
  //   move_group_interface_gripper_.setMaxAccelerationScalingFactor(0.05);
  //   moveit::planning_interface::MoveGroupInterface::Plan plan;
  //   bool success = (move_group_interface_gripper_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  //   if (success)
  //   {
  //     move_group_interface_.execute(plan);
  //     return true;
  //   }
  //   else
  //   {
  //     RCLCPP_ERROR(node_->get_logger(), "Planning failed for gripper!");
  //     return false;
  //   }
  // }




  // bool RobotControl::graspObject(std::string object_model, std::string object_link)
  // {
  //     auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
      
  //     request->model1_name = "ur";              // Name of the first model.
  //     request->link1_name = "wrist_3_link";     // Name of the link in the first model.
  //     request->model2_name = object_model;      // Name of the second model.
  //     request->link2_name = object_link;        // Name of the link in the second model.

  //     while (!gz_link_attacher_client_->wait_for_service(std::chrono::seconds(3))) 
  //     {
  //       if (!rclcpp::ok()) 
  //       {
  //         RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for link attacher service. Exiting.");
  //         return false;
  //       }
  //       RCLCPP_INFO(node_->get_logger(), "Link attacher service not available, waiting again...");
  //     }

  //     auto future_result = gz_link_attacher_client_->async_send_request(request);

  //     if (future_result.wait_for(std::chrono::seconds(3)) == std::future_status::ready) 
  //     {
  //         auto response = future_result.get();
  //         // Process the response
  //         RCLCPP_INFO_STREAM(node_->get_logger(), "Gripper successfully grasped " << object_model);
  //     } 
  //     else 
  //     {
  //         RCLCPP_ERROR_STREAM(node_->get_logger(), "Gripper failed to grasp " << object_model);
  //         return false;
  //     }
  //     return true;
  // }

  // bool RobotControl::releaseObject(std::string object_model, std::string object_link)
  // {
  //     auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
      
  //     request->model1_name = "ur";              // Name of the first model.
  //     request->link1_name = "wrist_3_link";     // Name of the link in the first model.
  //     request->model2_name = object_model;      // Name of the second model.
  //     request->link2_name = object_link;        // Name of the link in the second model.

  //     while (!gz_link_detacher_client_->wait_for_service(std::chrono::seconds(3))) 
  //     {
  //       if (!rclcpp::ok()) 
  //       {
  //         RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for link detacher service. Exiting.");
  //         return false;
  //       }
  //       RCLCPP_INFO(node_->get_logger(), "Link detacher service not available, waiting again...");
  //     }

  //     auto future_result = gz_link_detacher_client_->async_send_request(request);

  //     if (future_result.wait_for(std::chrono::seconds(3)) == std::future_status::ready) 
  //     {
  //         auto response = future_result.get();
  //         // Process the response
  //         RCLCPP_INFO_STREAM(node_->get_logger(), "Gripper successfully released " << object_model);
  //     } 
  //     else 
  //     {
  //       RCLCPP_ERROR_STREAM(node_->get_logger(), "Griper failed to release " << object_model);
  //       return false;
  //     }
  //     return true;
  // }

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

}