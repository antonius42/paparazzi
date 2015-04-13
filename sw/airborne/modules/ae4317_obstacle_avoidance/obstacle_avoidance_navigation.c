 /**
 * @file modules/computer_vision/obstacle_avoidance_navigation.c
 * @brief Navigation commands for obstacle avoidance
 *
 * Functions for updating a waypoint, and stopping (switching to a different flight plan block)
 * NOTE: this code needs a specific flight plan, in order to use the correct blocks and waypoint for navigation
 * 
 * AE4317 'Autonomous Flight of MAVs' - Obstacle Avoidance Competition
 * Delft University of Technology
 * Faculty of Aerospace Engineering
 * Department of Control & Simulation
 */

#include "obstacle_avoidance_navigation.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "std.h"
#include "stdio.h"
#include "math/pprz_algebra_int.h"
#include "generated/flight_plan.h"

/* Function for updating waypoint position */
void obstacle_avoidance_update_waypoint(int32_t heading_change, int32_t displacement)
{
	int32_t s_heading, c_heading, heading_new;  
	struct EnuCoor_i pos;

	// Indentify current position
	pos = *stateGetPositionEnu_i();
	VECT2_COPY(waypoints[WP_FROM], pos);

	// Add heading change to current heading, and normalize total heading
	heading_new = nav_heading + heading_change;
	INT32_ANGLE_NORMALIZE(heading_new);

	// Determine sine/cosine components of displacement, add them to current position
	PPRZ_ITRIG_SIN(s_heading, heading_new);
    PPRZ_ITRIG_COS(c_heading, heading_new);
	pos.x += displacement * s_heading/100;
	pos.y += displacement * c_heading/100;
	pos.z = nav_altitude;

	// Update waypoint and heading
	nav_move_waypoint(WP_TO, &pos);
	nav_set_heading_towards_waypoint(WP_TO);
}


/* Functions for stop commands */
void obstacle_avoidance_stop_go_left(void)
{
    GotoBlock(8); // Set corresponding navigation block. TODO make flight plan independent
}

void obstacle_avoidance_stop_go_right(void)
{
    GotoBlock(9); // Set corresponding navigation block. TODO make flight plan independent
}
