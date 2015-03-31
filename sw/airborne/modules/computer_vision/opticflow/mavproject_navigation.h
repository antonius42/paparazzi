/*
 *  MAVproject code for optical flow based navigation
 *  Developed for the AE4317 Autonomous Flight of MAV's. Implements obstacle avoidance based on
 *  vision from the ARdrone's frontal camera.
 */

#ifndef MAVPROJECT_NAVIGATION_H
#define MAVPROJECT_NAVIGATION_H

#include "std.h"


void obstacle_avoidance_update_waypoint(int32_t heading_change, int32_t displacement);
void obstacle_avoidance_stop_go_left(void);
void obstacle_avoidance_stop_go_right(void);

#endif
