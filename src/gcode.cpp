#include "sketch_bot_core/gcode.h"
#include <iostream>

namespace gcode
{

  Gcode::Gcode()
  {

  }

  Gcode::~Gcode()
  {

  }

  void Gcode::setOrigin(double x, double y, double z)
  {
    x_origin_ = x; 
    y_origin_ = y; 
    z_origin_ = z;
  }

  geometry_msgs::msg::PoseArray Gcode::toPoseArray(std::string gcode_file_path)
  {
    geometry_msgs::msg::PoseArray pose_array;
    std::cout<<"Loading gcode file with absolute path: "<<gcode_file_path<<std::endl;
    std::ifstream gcode_file; // input file
    char* gcode_file_path_char = &gcode_file_path[0];
    gcode_file.open(gcode_file_path_char);
    if (!gcode_file) 
      std::cerr<<"Unable to open gcode file"<<std::endl;

    std::string line_txt;
    std::string z_global="0.0"; //changes to new value of z_string once a new Z coordinate is encountered
    std::string x_global="0.0";std::string y_global="0.0"; std::string e_global="0.0";std::string f_global="0.0";

    std::vector<std::string> regex_str = {"X[-0-9]*\\.*[0-9]*","Y[-0-9]*\\.*[0-9]*","Z[-0-9]*\\.*[0-9]*","E[-0-9]*\\.*[0-9]*","F[0-9]*\\.*[0-9]*"};

    std::string x_string; std::string y_string; std::string z_string;  std::string G0 = "G0";
    std::string e_string; std::string f_string; std::string match_str; std::string G1 = "G1"; std::string G92 = "G92";
    int line_counter = 1;

    while (std::getline(gcode_file,line_txt))
    {
      if (line_txt.substr(0,2)==G0||line_txt.substr(0,2)==G1||line_txt.substr(0,3)==G92)
      {
        for (std::size_t i = 0; i < regex_str.size(); ++i){
          std::regex reg(regex_str[i]);
          std::sregex_iterator it(line_txt.begin(), line_txt.end(), reg);
          std::sregex_iterator it_end;
          while(it != it_end) 
          {
            std::smatch match = *it;
            match_str = match.str();
            ++it;
          }
          if (i==0)
          {
            x_string=match_str;
            if(!x_string.empty()){x_global = x_string.substr(1);}
          }
          else if (i==1)
          {
            y_string=match_str;
            if(!y_string.empty()){y_global = y_string.substr(1);}
          }
          else if (i==2)
          {
            z_string=match_str;
            if(!z_string.empty()){z_global = z_string.substr(1);}
          }
          else if (i==3)
          {
            e_string=match_str;
            if(!e_string.empty()){e_global = e_string.substr(1);}
          }
          else if (i==4)
          {
            f_string=match_str;
            if(!f_string.empty()){f_global = f_string.substr(1);}
          }
          match_str.clear();
        }
        double float_x = std::stod(x_global); 
        double float_y = std::stod(y_global); 
        double float_z = std::stod(z_global);

        if (line_txt.substr(0,2)==G0)
        {
          float_z = 10;
        }
        // double float_e = std::stod(e_global);
        // double float_f = std::stod(f_global);
        // std::cout<<"["<<line_counter<<"]: "<<float_x <<", "<<float_y<<", "<<float_z<<std::endl; 
        geometry_msgs::msg::Pose pose = createROSPoseMsg(float_x*0.001 + x_origin_, float_y*0.001 + y_origin_, float_z*0.001 + z_origin_);
        pose_array.poses.push_back(pose);
        
      } //if
      ++line_counter;
    } //while loop

    gcode_file.close();
    return pose_array;
  }



  geometry_msgs::msg::Pose Gcode::createROSPoseMsg(double x, double y, double z)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1.0;
    return pose;
  }

}