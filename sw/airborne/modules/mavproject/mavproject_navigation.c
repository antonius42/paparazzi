/*
 * MAVproject main source code (navigation)
 *
 */

#include "mavproject_navigation.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "std.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_passthrough.h"


void obstacle_avoidance_change_heading(int32_t heading_change)
{
    nav_set_heading_deg(nav_heading - heading_change);
    //set_yaw_sp_to_heading(); (may be necessary only if yaw is not changed automatically)
}

void set_yaw_sp_to_angle(int32_t angle)
{
    stab_att_sp_euler.psi = angle;
}

void set_yaw_sp_to_heading(void)
{
    set_yaw_sp_to_angle(nav_heading);
}

void obstacle_avoidance_stop(void)
{
    // not inplemented yet
}
