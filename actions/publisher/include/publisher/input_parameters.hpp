#pragma once

#include <string>
#include <vector>

struct naviation_pose_2d_t
{
  double x;
  double y;
  double yaw;
};

struct input_parameters_t
{
  std::string topic;
  std::string robot_name;
  std::string reference_frame;
  naviation_pose_2d_t naviation_pose_2d;
};

