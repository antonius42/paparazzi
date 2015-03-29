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

/// Data from thread to module
struct CVresults {
  int cnt;          // Number of processed frames

//REMOVED_MAV  float Velx;       // Velocity as measured by camera
//REMOVED_MAV   float Vely;
  int flow_count;

//REMOVED_MAV   float cam_h;      // Debug parameters
  int count;
//REMOVED_MAV   float OFx, OFy;//REMOVED_MAV  , dx_sum, dy_sum;
  float diff_roll;
  float diff_yaw; // Changed pitch to yaw
  float FPS;
  float OFtotal; //Total optic flow, has been added. 
  float OFtotalL; // Total optic flow on the left has been added.
  float OFtotalR; // Total optic flow on the right has been added.
  float OFFlessZone; // Size of the featureless zone has been added.
  float OFFlessZonePos; // Center of the featureless zone has been added.
};

/// Data from module to thread
struct PPRZinfo {
  int cnt;        // IMU msg counter
  float phi;      // roll [rad]
  float psi;    // yaw [rad]   Changed pitch to yaw.
  float agl;      // height above ground [m]
};

#endif
