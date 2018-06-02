#ifndef PERCEPTION_HPP_INCLUDED
#define PERCEPTION_HPP_INCLUDED

#include "helper.hpp"
#include "main.hpp"
#include <climits>
#include <cmath>
#include <algorithm>

// struct LaserData
// {
//   double range_min;
//   double range_max;
//
//   double angle_min;
//   double angle_max;
//   double angle_increment;
//
//   std::vector<float> ranges;
//
//   double timestamp;
// };

// struct OdometryData
// {
//   double x;
//   double y;
//   double a;
//   double timestamp;
// };

bool perception(struct ROB_state *rob, struct WM_state *wm);
bool perception_drive_towards_wall(struct ROB_state *rob,struct WM_state *wm);
bool filtered_error_measurement_at_angle(struct ROB_state *rob, struct WM_state *wm);
bool perception_follow_wall(struct ROB_state *rob,struct WM_state *wm);
bool average_of_local_distances(std::vector<float> *distances, const float * threshold, Interval * interval, float * result);

void initialize_wall_following(struct ROB_state *rob,struct WM_state *wm);
bool local_neighbourhood(struct ROB_state *rob, std::vector<float> *distances, std::vector<std::vector<float> > *vec_distances, std::vector<int> *start_index);

#endif
