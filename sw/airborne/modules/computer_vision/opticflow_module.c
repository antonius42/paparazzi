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


#include "opticflow_module.h"

#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "subsystems/abi.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "opticflow/mavproject_navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/flight_plan.h"

/* default sonar/agl to use in opticflow visual_estimator */
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OPTICFLOW_AGL_ID);

// Define the downscale factor for the frontal camera
#include "lib/vision/image.h"
#define img_downscale_factor 4

// Define waypoint movement frequency
#define NAV_UPDATE_COUNT 8
// Define navigation settings/thresholds
#define NAV_HEADING_CHANGE 500
#define NAV_WAYPOINT_DISPLACEMENT 10
#define NAV_TURN_THRESHOLD_OF 1500
#define NAV_STOP_THRESHOLD_OF 4000
#define NAV_TURN_THRESHOLD_FZ 80
#define NAV_STOP_THRESHOLD_FZ 100

/* The main opticflow variables */
static struct opticflow_t opticflow;                //< Opticflow calculations
static struct opticflow_result_t opticflow_result;  //< The opticflow result		//defined in inter_thread_data.h
static struct opticflow_state_t opticflow_state;    //< State of the drone to communicate with the opticflow
static struct v4l2_device *opticflow_dev;           //< The opticflow camera V4L2 device
static abi_event opticflow_agl_ev;                  //< The altitude ABI event
static pthread_t opticflow_calc_thread;             //< The optical flow calculation thread
static bool_t opticflow_got_result;                 //< When we have an optical flow calculation
static pthread_mutex_t opticflow_mutex;             //< Mutex lock fo thread safety

/* Navigation housekeeping*/
int nav_counter;                                    //< Counter that regulates updating of the waypoint
float OF_left_av, OF_right_av, OF_total_av;         //< Average values of measured optical flow
float OF_fz_av, OF_fz_pos_av;

/* Static functions */
static void *opticflow_module_calc(void *data);                   //< The main optical flow calculation thread
static void opticflow_agl_cb(uint8_t sender_id, float distance);  //< Callback function of the ground altitude


/**
 * Initialize the optical flow module for the bottom camera
 */
void opticflow_module_init(void)
{
  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(OPTICFLOW_AGL_ID, &opticflow_agl_ev, opticflow_agl_cb);

  // Set the opticflow state to 0
  opticflow_state.phi = 0;
  opticflow_state.psi = 0;		// Tobias: added psi to the state
  opticflow_state.theta = 0;
  opticflow_state.agl = 0;

  // Initialize navigation counters/averages
  nav_counter = 0;
  OF_left_av  = 0.0;
  OF_right_av = 0.0;
  OF_total_av = 0.0;
  OF_fz_av = 0.0;
  OF_fz_pos_av = 0.0;
  
  // Initialize the opticflow calculation
//  opticflow_calc_init(&opticflow, 320, 240);		// Tobias: Use this for the downward camera
//  opticflow_calc_init(&opticflow, 1280, 720);		// Tobias: Use this for the frontal camera
  opticflow_calc_init(&opticflow, 1280/img_downscale_factor, 720/img_downscale_factor);	// Tobias: Use this for frontal camera scaling.
  opticflow_got_result = FALSE;

  /* Try to initialize the video device */
  //v4l2_init_subdev("/dev/v4l-subdev0", 0, 1, V4L2_MBUS_FMT_UYVY8_2X8, 320, 240);
//  opticflow_dev = v4l2_init("/dev/video2", 320, 240, 60); //TODO: Fix defines          // Tobias: Use this for downwards camera
  opticflow_dev = v4l2_init("/dev/video1", 1280, 720, 60); //TODO: Fix defines		// Tobias: Use this for frontal camera
  
  // Set navigation max speed
  guidance_h_SetMaxSpeed(0.4);
  
  if (opticflow_dev == NULL) {
    printf("[opticflow_module] Could not initialize the video device\n");
  }
}

/**
 * Update the optical flow state for the calculation thread
 * and update the stabilization loops with the newest result
 */
void opticflow_module_run(void)
{
  pthread_mutex_lock(&opticflow_mutex);
  // Send Updated data to thread
  opticflow_state.phi = stateGetNedToBodyEulers_f()->phi;
  opticflow_state.theta = stateGetNedToBodyEulers_f()->theta;
  opticflow_state.psi = stateGetNedToBodyEulers_f()->psi;		// Tobias: added psi to the state
  
  pthread_mutex_unlock(&opticflow_mutex);
}

/**
 * Start the optical flow calculation
 */
void opticflow_module_start(void)
{
  // Check if we are not already running
  if(opticflow_calc_thread != 0) {
    printf("[opticflow_module] Opticflow already started!\n");
    return;
  }

  // Create the opticalflow calculation thread
  int rc = pthread_create(&opticflow_calc_thread, NULL, opticflow_module_calc, NULL);
  if (rc) {
    printf("[opticflow_module] Could not initialize opticflow thread (return code: %d)\n", rc);
  }
}

/**
 * Stop the optical flow calculation
 */
void opticflow_module_stop(void)
{
  // Stop the capturing
  v4l2_stop_capture(opticflow_dev);

  // TODO: fix thread stop
}

/**
 * The main optical flow calculation thread
 * This thread passes the images trough the optical flow
 * calculator based on Lucas Kanade
 */
#include "errno.h"
static void *opticflow_module_calc(void *data __attribute__((unused))) {
  // Start the streaming on the V4L2 device
  if(!v4l2_start_capture(opticflow_dev)) {
    printf("[opticflow_module] Could not start capture of the camera\n");
    return 0;
  }

  /* Main loop of the optical flow calculation */
  while(TRUE) {
    // Try to fetch an image
//    struct image_t img;
//    v4l2_image_get(opticflow_dev, &img);

// ***************** Start downscale the Image *****************
// Note: When enabling this downscaling, see opticflow_calc_init(&opticflow, 1280, 720), line 82. must be 1280/4, 720/4

    struct image_t img_orig;		// Tobias: Declare img_orig
    v4l2_image_get(opticflow_dev, &img_orig);		// Tobias: Get the image from the camera device and store in img_org (1280,720)

    struct image_t img;	// Tobias: Declare img
    image_create(&img, img_orig.w/img_downscale_factor, img_orig.h/img_downscale_factor, IMAGE_YUV422);
//    printf("w=%i h=%i 	\n",img.w,img.h);
    
//printf("This is the run run 1	\n");
    image_yuv422_downsample(&img_orig, &img, img_downscale_factor);	// Tobias: Downsample img_orig with factor 4, store result in img (320,180)
//printf("This is the run run 2	\n");

//printf("orig w=%i h=%i	\n",img_orig.w,img_orig.h);
//printf("img  w=%i h=%i	\n",img.w,img.h);

//    img = img_orig;



// ***************** End downscale the Image *****************

    // Copy the state
    pthread_mutex_lock(&opticflow_mutex);
    struct opticflow_state_t temp_state;
    memcpy(&temp_state, &opticflow_state, sizeof(struct opticflow_state_t));
    pthread_mutex_unlock(&opticflow_mutex);

    // Do the optical flow calculation
    struct opticflow_result_t temp_result;
    opticflow_calc_frame(&opticflow, &temp_state, &img, &temp_result);

    // Copy the result if finished
    pthread_mutex_lock(&opticflow_mutex);
    memcpy(&opticflow_result, &temp_result, sizeof(struct opticflow_result_t));
    opticflow_got_result = TRUE;
    pthread_mutex_unlock(&opticflow_mutex);

    // Free the image
    image_free(&img);
    v4l2_image_free(opticflow_dev, &img_orig);

    // On-line averaging of optical flow
    nav_counter++;
    OF_left_av = OF_left_av/nav_counter*(nav_counter-1) + opticflow_result.tot_of_left/nav_counter;
    OF_right_av = OF_right_av/nav_counter*(nav_counter-1) + opticflow_result.tot_of_right/nav_counter;
    OF_total_av = OF_total_av/nav_counter*(nav_counter-1) + opticflow_result.tot_of/nav_counter;
    OF_fz_av = OF_fz_av/nav_counter*(nav_counter-1) + (1.0*opticflow_result.of_featurelesszone)/nav_counter;
    OF_fz_pos_av = OF_fz_pos_av/nav_counter*(nav_counter-1) + (1.0*opticflow_result.of_featurelesszone_pos)/nav_counter;
    // Perform navigation commands
    if (nav_counter == NAV_UPDATE_COUNT) {
      int32_t heading_change = 0;
      printf("Featureless zone size = %0.3f, position %0.3f \n",OF_fz_av, OF_fz_pos_av);
      printf("Total of left = %0.3f	\n",OF_left_av);
      printf("Total of right = %0.3f	\n",OF_right_av);    
      printf("Total of total = %0.3f	\n",OF_total_av);
      // Reset the counter
      nav_counter = 0;
      // Turn if significant optical flow is found
      if (nav_block == 6) {
      	if (OF_total_av > NAV_TURN_THRESHOLD_OF) {
        	if (OF_left_av > OF_right_av) {
          	// Turn right
          	heading_change = NAV_HEADING_CHANGE;
          	printf("Going right (OF) \n"); 
        	}
        	if (OF_left_av < OF_right_av) {
        	  // Turn left
        	  heading_change = -NAV_HEADING_CHANGE;
        	  printf("Going left (OF)\n");
        	}
      	}
      	if (OF_fz_av > NAV_TURN_THRESHOLD_FZ) {
        	if (OF_fz_pos_av < 320/2) {
          	// Turn right
          	heading_change = NAV_HEADING_CHANGE;
          	printf("Going right (FZ)\n"); 
        	}
        	if (OF_fz_pos_av > 320/2) {
        	  // Turn left
        	  heading_change = -NAV_HEADING_CHANGE;
        	  printf("Going left (FZ)\n");
        	}
      	}
      	
      	// Stop if too close to object
      	if (OF_total_av > NAV_STOP_THRESHOLD_OF)  {
        	printf("STOP AND TURN (OF) \n");
        	obstacle_avoidance_stop();
      	}
      	if (OF_fz_av > NAV_STOP_THRESHOLD_FZ) {
      		printf("STOP AND TURN (FZ) \n");
        	obstacle_avoidance_stop();
      	}
      }
      obstacle_avoidance_update_waypoint(heading_change, NAV_WAYPOINT_DISPLACEMENT); 
    }
  }
}



/**
 * Get the altitude above ground of the drone
 * @param[in] sender_id The id that send the ABI message (unused)
 * @param[in] distance The distance above ground level in meters
 */
static void opticflow_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  // Update the distance if we got a valid measurement
  if (distance > 0) {
    opticflow_state.agl = distance;
  }
}
