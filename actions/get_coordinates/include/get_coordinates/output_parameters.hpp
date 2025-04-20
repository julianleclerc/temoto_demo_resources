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
  position_t position;
  orientation_t orientation;
};

struct output_parameters_t
{
  pose_t pose;
};

