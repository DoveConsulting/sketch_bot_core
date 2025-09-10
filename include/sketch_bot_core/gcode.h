#ifndef GCODE_H
#define GCODE_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <fstream>
#include <regex>

namespace gcode
{

class Gcode
{
public:
  Gcode();

  ~Gcode();

  geometry_msgs::msg::PoseArray toPoseArray(std::string gcode_file_path);
  void setOrigin(double x, double y, double z);

private:
  geometry_msgs::msg::Pose createROSPoseMsg(double x, double y, double z);
  double x_origin_ = 0.0;
  double y_origin_ = 0.0;
  double z_origin_ = 0.0;

};



}
#endif