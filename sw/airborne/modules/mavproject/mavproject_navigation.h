/*
 *  MAVproject code for optical flow based navigation
 *  Developed for the AE4317 Autonomous Flight of MAV's. Implements obstacle avoidance based on
 *  vision from the ARdrone's frontal camera.
 */

#ifndef MAVPROJECT_NAVIGATION_H
#define MAVPROJECT_NAVIGATION_H

#include "std.h"


void obstacle_avoidance_change_heading(int32_t heading_change);
void set_yaw_sp_to_angle(int32_t angle);
void set_yaw_sp_to_heading(void);
void obstacle_avoidance_stop(void);

#endif
