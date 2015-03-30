/*
 * Copyright (C) 2014 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow_module.c
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */


#include "opticflow_module.h" // Has been edited
#include "opticflow/mavproject_navigation.h"
#include "stdio.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/flight_plan.h"
#include "math/pprz_algebra_int.h"


#define RESET_TIME 0.1
#define ITERATIONS_PER_SECOND 500

int counter;

void opticflow_module_init(void)
{
	counter = 0;
	guidance_h_SetMaxSpeed(0.5);
}


void opticflow_module_run(void)
{
  counter++;
  if (counter == RESET_TIME * ITERATIONS_PER_SECOND) {
    obstacle_avoidance_update_waypoint(0, 10); 
	counter = 0;
  }
}

void opticflow_module_start(void)
{
  //
}

void opticflow_module_stop(void)
{
	//
}
