#ifndef GCODE_H
#define GCODE_H

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace gcode
{

class Gcode
{
public:
  Gcode();

  ~Gcode();

private:
  geometry_msgs::msg::Pose createROSPoseMsg(double x, double y, double z, double roll, double pitch, double yaw);
};



}
#endif