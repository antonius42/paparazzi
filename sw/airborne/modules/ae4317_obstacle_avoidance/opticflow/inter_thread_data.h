/*
 * Copyright (C) 2015 The Paparazzi Community
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
 * @file modules/computer_vision/opticflow/inter_thread_data.h
 * @brief Inter-thread data structures.
 *
 * Data structures used to for inter-thread communication via Unix Domain sockets.
 */


#ifndef _INTER_THREAD_DATA_H
#define _INTER_THREAD_DATA_H

/* The result calculated from the opticflow */
struct opticflow_result_t {
  float fps;              //< Frames per second of the optical flow calculation
  uint16_t corner_cnt;    //< The amount of coners found by FAST9
  uint16_t tracked_cnt;   //< The amount of tracked corners

  float flow_x;           //< Flow in x direction from the camera (in subpixels)
  int16_t flow_y;         //< Flow in y direction from the camera (in subpixels)
  float flow_der_x;       //< The derotated flow calculation in the x direction (in subpixels)
  
  float tot_of_left;	    //< Total optic flow left
  float tot_of_right;	    //< Total optic flow right
  float tot_of;	          //< Total optic flow
  
  int16_t of_featurelesszone;		  //< Maximum distance between to features.
  int16_t of_featurelesszone_pos;	//< Location of the featureless zone.
};

/* The state of the drone when it took an image */
struct opticflow_state_t {
  float phi;      //< roll [rad]
  float psi;      //< yaw [rad]		// Tobias: added psi to the state
  float theta;    //< pitch [rad]
  float agl;      //< height above ground [m]
};

#endif
