 /**
 * @file modules/computer_vision/obstacle_avoidance_navigation.h
 * @brief Navigation commands for obstacle avoidance
 *
 * Functions for updating a waypoint, and stopping (switching to a different flight plan block)
 * NOTE: this code needs a specific flight plan, in order to use the correct blocks and waypoint for navigation
 * 
 * AE4317 'Autonomous Flight of MAVs' - Obstacle Avoidance Competition Group 3
 * Delft University of Technology
 * Faculty of Aerospace Engineering
 * Department of Control & Simulation
 */

#ifndef OBSTACLE_AVOIDANCE_NAVIGATION_H
#define OBSTACLE_AVOIDANCE_NAVIGATION_H

#include "std.h"


void obstacle_avoidance_update_waypoint(int32_t heading_change, int32_t displacement);
void obstacle_avoidance_stop_go_left(void);
void obstacle_avoidance_stop_go_right(void);

#endif
