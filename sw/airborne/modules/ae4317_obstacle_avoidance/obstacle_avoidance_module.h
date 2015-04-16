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
 * @file modules/computer_vision/opticflow_module.h
 * @brief optical-flow based obstacle avoidance module 
 * 
 * Main code of the obstacle avoidance module based on optical flow/featureless zone.
 * Defines optical flow estimation thread, which also controls the navigation for obstacle avoidance.
 * 
 * AE4317 'Autonomous Flight of MAVs' - Obstacle Avoidance Competition Group 3
 * Delft University of Technology
 * Faculty of Aerospace Engineering
 * Department of Control & Simulation
 */

#ifndef OBSTACLE_AVOIDANCE_MODULE_H
#define OBSTACLE_AVOIDANCE_MODULE_H

#include "opticflow/opticflow_calculator.h"

// Module functions
extern void obstacle_avoidance_module_init(void);
extern void obstacle_avoidance_module_run(void);
extern void obstacle_avoidance_module_start(void);
extern void obstacle_avoidance_module_stop(void);

#endif /* OBSTACLE_AVOIDANCE_MODULE_H */
