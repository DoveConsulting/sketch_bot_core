#include "sketch_bot_core/gcode.h"
#include <iostream>

namespace gcode
{

  Gcode::Gcode()
  {
    std::cout<<"GCODE Constructor============================================"<<std::endl;

  }

  Gcode::~Gcode()
  {

  }


  geometry_msgs::msg::Pose Gcode::createROSPoseMsg(double x, double y, double z, double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::Pose pose;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // tf2::Quaternion q;
    // q.setRPY(roll*(M_PI/180.0), pitch*(M_PI/180.0), yaw*(M_PI/180.0));
    // geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);
    // pose.orientation = msg_quat;

    return pose;
  }

}