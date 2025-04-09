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
  double x;
  double y;
  double z;
  double w;
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
  std::string planning_group;
  std::string target_link;
  pose_t pose;
  double max_planning_time;
};

