/*
 * MAVproject main source code
 *
 */

#include "mavproject_module.h"
// BAS: I split up the functionalities into two files so that we can work on it simultaneously
#include "mavproject_optical_flow.h"
#include "mavproject_navigation.h"
// Other includes here. 


void init_autonomous_flight(void)
{
    // initialize variables here
}

void run_autonomous_flight(void)
{
    // main loop to be executed during flight
    // When they work, these functions are called (or something similar):
    // mavproject_detect_optical_flow();
    // mavproject_navigate();	
}

void start_autonomous_flight(void)
{
    // executes this code at start of the main loop
}

void stop_autonomous_flight(void)
{
    // executes this code at end of the main loop
}

