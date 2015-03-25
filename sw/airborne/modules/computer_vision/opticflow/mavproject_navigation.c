/*
 * MAVproject main source code (navigation)
 * Needs a waypoint to be defined as WP_STDBY
 */

#include "mavproject_navigation.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "std.h"
#include "stdio.h"

#include "generated/flight_plan.h"


void obstacle_avoidance_update_waypoint(int32_t heading_change, int32_t displacement)
{
	int32_t s_heading, c_heading;  
	VECT2_COPY(waypoints[WP_FROM], *stateGetPositionEnu_i());  
	PPRZ_ITRIG_SIN(s_heading, nav_heading + heading_change);
    PPRZ_ITRIG_COS(c_heading, nav_heading + heading_change);
    waypoints[WP_TO].x = displacement * c_heading;
    waypoints[WP_TO].y = displacement * s_heading;
	nav_set_heading_towards_waypoint(WP_TO);
    printf("Waypoint TO position updated to (%i,%i)", waypoints[WP_TO].x,waypoints[WP_TO].y);
}

void obstacle_avoidance_stop(void)
{
    NavSetWaypointHere(WP_STDBY);
}
