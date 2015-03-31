/*
 * MAVproject main source code (navigation)
 * Needs a waypoint to be defined as WP_STDBY
 */

#include "mavproject_navigation.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "std.h"
#include "stdio.h"
#include "math/pprz_algebra_int.h"
#include "generated/flight_plan.h"


void obstacle_avoidance_update_waypoint(int32_t heading_change, int32_t displacement)
{
	int32_t s_heading, c_heading, heading_new;  
	struct EnuCoor_i pos;
	pos = *stateGetPositionEnu_i();
	VECT2_COPY(waypoints[WP_FROM], pos);
	heading_new = nav_heading + heading_change;
	INT32_ANGLE_NORMALIZE(heading_new);
	PPRZ_ITRIG_SIN(s_heading, heading_new);
    PPRZ_ITRIG_COS(c_heading, heading_new);
	pos.x += displacement * s_heading/100;
	pos.y += displacement * c_heading/100;
	pos.z = nav_altitude;
	nav_move_waypoint(WP_TO, &pos);
	nav_set_heading_towards_waypoint(WP_TO);
}

void obstacle_avoidance_stop_go_left(void)
{
    GotoBlock(8); // IMPORTANT: this number changes when we update the flight plan
}

void obstacle_avoidance_stop_go_right(void)
{
    GotoBlock(9); // IMPORTANT: this number changes when we update the flight plan
}
