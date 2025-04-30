#pragma once

#include <string>
#include <vector>

struct position_t
{
  double x;
  double y;
  double z;
};

struct orientation_t
{
  double r;
  double p;
  double y;
};

struct pose_t
{
  std::string frame_id;
  position_t position;
  orientation_t orientation;
};

struct input_parameters_t
{
  std::string robot_name;
  std::string location;
  pose_t pose;
};

