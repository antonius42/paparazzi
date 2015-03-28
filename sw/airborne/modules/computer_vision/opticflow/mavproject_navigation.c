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
#include "subsystems/datalink/downlink.h"


void obstacle_avoidance_update_waypoint(int32_t heading_change, int32_t displacement)
{
	int32_t s_heading, c_heading;  
	VECT2_COPY(waypoints[WP_FROM], *stateGetPositionEnu_i());  
	PPRZ_ITRIG_SIN(s_heading, nav_heading + heading_change);
    PPRZ_ITRIG_COS(c_heading, nav_heading + heading_change);
    waypoints[WP_TO].x = waypoints[WP_FROM].x + displacement * c_heading/50;
    waypoints[WP_TO].y = waypoints[WP_FROM].y + displacement * s_heading/50;
   // waypoints[WP_TO].z = //
	nav_set_heading_towards_waypoint(WP_TO);
	printf("Heading setting: %i \n", nav_heading);
	printf("Waypoint FROM position updated to (%i,%i) \n", waypoints[WP_FROM].x,waypoints[WP_FROM].y);
    printf("Waypoint TO position updated to (%i,%i) \n", waypoints[WP_TO].x,waypoints[WP_TO].y);
	//DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel,DefaultDevice, (uint8_t *)WP_FROM, &(waypoints[WP_FROM].x), &(waypoints[WP_FROM].y),&(waypoints[WP_FROM].z));
	//DOWNLINK_SEND_WP_MOVED_ENU(DefaultChannel,DefaultDevice, (uint8_t *)WP_TO, &(waypoints[WP_TO].x), &(waypoints[WP_TO].y),&(waypoints[WP_TO].z));
	
}

void obstacle_avoidance_stop(void)
{
    GotoBlock(11); // IMPORTANT: this number changes when we update the flight plan
}
