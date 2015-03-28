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
 * @file modules/computer_vision/opticflow/visual_estimator.c
 * @brief Estimate velocity from optic flow.
 *
 * Using sensors from vertical camera and IMU of Parrot AR.Drone 2.0.
 *
 * Warning: all this code is called form the Vision-Thread: do not access any autopilot data in here.
 */

#include "std.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Own Header
#include "visual_estimator.h" // Has been edited

// Computer Vision
#include "opticflow/optic_flow_int.h" // Has been edited
#include "opticflow/fast9/fastRosten.h"

// for FPS
#include "modules/computer_vision/cv/framerate.h"
int disp_counter;  // Added a counter for the display

// For resize
#include "modules/computer_vision/cv/resize.h"

// Local variables
struct visual_estimator_struct	//Create struct of type visual_estimator_struct
{
  // Image size
  unsigned int imgWidth;
  unsigned int imgHeight;

  // Images
  uint8_t *prev_frame;
  uint8_t *gray_frame;
  uint8_t *prev_gray_frame;
  #define downsize_factor 4	// downscaling factor of the image

  // Initialization
  int old_img_init;

  // Store previous
  float prev_yaw;
  float prev_roll;
} visual_estimator;	// Create an alias named visual_estimator, it contains a.o. the pointers *prev_frame *gray_frame and *prev_gray_frame

// ARDrone Vertical Camera Parameters Change this to Frontal Camera Parameters
#define FOV_H 0.67020643276 // Change this to Frontal Camera Parameters
#define FOV_W 0.89360857702 // Change this to Frontal Camera Parameters
//REMOVED_MAV #define Fx_ARdrone 343.1211
//REMOVED_MAV #define Fy_ARdrone 348.5053

// Corner Detection
#define MAX_COUNT 100

// Flow Derotation
#define FLOW_DEROTATION


// Called by plugin
void opticflow_plugin_init(unsigned int w, unsigned int h, struct CVresults *results)
{
  // Initialize variables
  visual_estimator.imgWidth = w;
  visual_estimator.imgHeight = h;
  
  visual_estimator.gray_frame = (unsigned char *) calloc(w * h, sizeof(uint8_t));	// Resized
  visual_estimator.prev_frame = (unsigned char *) calloc(w * h * 2, sizeof(uint8_t));	// Resized
  visual_estimator.prev_gray_frame = (unsigned char *) calloc(w * h, sizeof(uint8_t));	// Resized

  visual_estimator.old_img_init = 1;
  visual_estimator.prev_yaw = 0.0;
  visual_estimator.prev_roll = 0.0;

//REMOVED_MAV   results->OFx = 0.0;
//REMOVED_MAV   results->OFy = 0.0;
//REMOVED_MAV  results->dx_sum = 0.0;
//REMOVED_MAV   results->dy_sum = 0.0;
  results->diff_roll = 0.0;
  results->diff_yaw = 0.0;
//REMOVED_MAV   results->cam_h = 0.0;
//REMOVED_MAV   results->Velx = 0.0;
//REMOVED_MAV   results->Vely = 0.0;
  results->flow_count = 0;
  results->cnt = 0;
  results->count = 0;
  results->OFtotal = 0; //Total optic flow, has been added. 
  results->OFtotalL = 0; // Total optic flow on the left has been added.
  results->OFtotalR = 0; // Total optic flow on the right has been added.
  results->OFFlessZone = 0; // Size of the featureless zone has been added.
  results->OFFlessZonePos = 0; // Center of the featureless zone has been added.

  disp_counter = 0; // Display counter

  framerate_init();
}

void opticflow_plugin_run(unsigned char *frame, struct PPRZinfo* info, struct CVresults *results)
{
  // Corner Tracking
  // Working Variables
  int max_count = 25;
  int borderx = 24, bordery = 24;
  int x[MAX_COUNT], y[MAX_COUNT];
  int new_x[MAX_COUNT], new_y[MAX_COUNT];
  int status[MAX_COUNT];
  int dx[MAX_COUNT];
  int w = visual_estimator.imgWidth;
  int h = visual_estimator.imgHeight;
  
//  printf("downsize_f = %i, w = %i, h = %i	\n",downsize_factor,w,h);


// INSERT the resizing here...
  // Resize image in memory located at pointer 'frame'
  // frame == current frame, prev_frame == previous frame
/*
  struct img_struct new_frame;	// initialize the image structure for the new frame new_frame
  new_frame.w = w / downsize_factor;	// scale the new width with the downsize factor
  new_frame.h = h / downsize_factor;	// scale the new height with the downsize factor
  if (downsize_factor != 1) {
    new_frame.buf = (uint8_t *)malloc(new_frame.w * new_frame.h * 2);	// reserve memory space for the frame
  }
  
    
  printf("downsize_f = %i, w = %i, h = %i	\n",downsize_factor,new_frame.w,new_frame.h);

  // Resize the image
  if (downsize_factor != 1) {
    struct img_struct cur_frame;	// initialize the image structure for the current frame: cur_frame
    cur_frame.buf = frame;		// store the frame in the buffer, probably the image in [0-255] values ??
    cur_frame.w = w;	// set the current frame width
    cur_frame.h = h;	// set the current frame height
    resize_uyuv(&cur_frame, &new_frame, downsize_factor);	// resize current frame using (adress of cur_frame, adress of new_frame and downsize_factor)
    
    
    // The new image is now stored in the memory at pointer new_frame.buf
    
    w = w / downsize_factor;	// Resize the image width variable
    h = h / downsize_factor;	// Resize the image height variable
  }
*/
  // Framerate Measuring
  results->FPS = framerate_run();

  if (visual_estimator.old_img_init == 1) {	// Run this for the first run (previous frame does not exist)
//    memcpy(visual_estimator.prev_frame, frame, w * h * 2);	// Copy the data in pointer frame to the memory at pointer visual_estimator.prev_frame
    memcpy(visual_estimator.prev_frame, frame, w * h * 2);	// Copy the data in pointer new_frame.buf to the memory at pointer visual_estimator.prev_frame
    CvtYUYV2Gray(visual_estimator.prev_gray_frame, visual_estimator.prev_frame, w, h);	// Create grayscale image
    visual_estimator.old_img_init = 0;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection
  int fast_threshold = 40; // original threshold = 20
  xyFAST *pnts_fast;
  pnts_fast = fast9_detect((const byte *)visual_estimator.prev_gray_frame, w, h, w,
                           fast_threshold, &results->count);
  if (results->count > MAX_COUNT) { results->count = MAX_COUNT; }
  for (int i = 0; i < results->count; i++) {
    x[i] = pnts_fast[i].x;
    y[i] = pnts_fast[i].y;
  }
  free(pnts_fast);

  if (disp_counter == 9){
  //printf("Number of features from Fast = %i \n",results->count);

    for (int i = 0; i < results->count; i++){
    //printf("x(%i)=%i,y(%i)=%i,",i+1,x[i],i+1,y[i]);
    }
    //printf(" \n");
  }

  // Remove neighboring corners
  const float min_distance = 3;
  float min_distance2 = min_distance * min_distance;
  int labelmin[MAX_COUNT];
  for (int i = 0; i < results->count; i++) {
    for (int j = i + 1; j < results->count; j++) {
      // distance squared:
      float distance2 = (x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]);
      if (distance2 < min_distance2) {
        labelmin[i] = 1;
      }
    }
  }

  int count_fil = results->count;  
  for (int i = results->count - 1; i >= 0; i--) {
    int remove_point = 0;

    if (labelmin[i]) {
      remove_point = 1;
    }

    if (remove_point) {
      for (int c = i; c < count_fil - 1; c++) {
        x[c] = x[c + 1];
        y[c] = y[c + 1];
      }
      count_fil--;
    }
  } 

  if (disp_counter == 9){
  //printf("Number of features after feature removal = %i \n",count_fil);
  }

  if (count_fil > max_count) { count_fil = max_count; }
  results->count = count_fil;



  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************
//  CvtYUYV2Gray(visual_estimator.gray_frame, frame, w, h);
  CvtYUYV2Gray(visual_estimator.gray_frame, frame, w, h);	// Used the pointer new_frame.buf to the resized image

  opticFlowLK(visual_estimator.gray_frame, visual_estimator.prev_gray_frame, x, y,
              count_fil, w, h, new_x, new_y, status, 5, 100);

  results->flow_count = count_fil;
  for (int i = count_fil - 1; i >= 0; i--) {
    int remove_point = 1;

    if (status[i] && !(new_x[i] < borderx || new_x[i] > (w - 1 - borderx) ||
                       new_y[i] < bordery || new_y[i] > (h - 1 - bordery))) {
      remove_point = 0;
    }

    if (remove_point) {
      for (int c = i; c < results->flow_count - 1; c++) {
        x[c] = x[c + 1];
        y[c] = y[c + 1];
        new_x[c] = new_x[c + 1];
        new_y[c] = new_y[c + 1];
      }
      results->flow_count--;
    }
  }

//REMOVED_MAV  results->dx_sum = 0.0;
//REMOVED_MAV  results->dy_sum = 0.0;

  // Optical Flow Computation
  for (int i = 0; i < results->flow_count; i++) {
    dx[i] = new_x[i] - x[i];
//    dy[i] = new_y[i] - y[i];
  }
/* REMOVED_MAV 
  // Median Filter
  if (results->flow_count) {
    quick_sort_int(dx, results->flow_count); // 11
    quick_sort_int(dy, results->flow_count); // 11

    results->dx_sum = (float) dx[results->flow_count / 2];
    results->dy_sum = (float) dy[results->flow_count / 2];
  } else {
    results->dx_sum = 0.0;
    results->dy_sum = 0.0;
  }
REMOVED_MAV */

  // Flow Derotation
  results->diff_yaw = (info->psi - visual_estimator.prev_yaw) * w / FOV_W; // Changed pitch to yaw and psi
  results->diff_roll = (info->phi - visual_estimator.prev_roll) * w / FOV_W;
  visual_estimator.prev_yaw = info->psi; // Changed to yaw and psi
  visual_estimator.prev_roll = info->phi;


// Change loop over the dy array.
float dx_t[MAX_COUNT];

// Added this loop
for (int i = 0; i < results->flow_count; i++){
#ifdef FLOW_DEROTATION
  if (results->flow_count) {
    dx_t[i] = dx[i] - (new_y[i] - h / 2) * results->diff_roll; // derotate the dx for roll
    dx_t[i] = dx_t[i] - results->diff_yaw; // derotate for yaw

    if ((dx_t[i] <= 0) != (dx[i] <= 0)) {
      dx_t[i] = 0;
    }
  } else {
    dx_t[i] = dx[i];
  }
#else
  dx_t[i] = dx[i];
#endif
}


// Calculate total optical flow
results->OFtotal = 0;

for (int i = 0; i < results->flow_count; i++){
  results->OFtotal = results->OFtotal + dx[i];
}

  disp_counter++;
  if (disp_counter == 10){
 // printf("Total OF %f \n", results->OFtotal);
 // printf("Number of features %i \n",results->flow_count);
  disp_counter = 0;
  }
// ADD total, total left, total right...

/*

float temp0, temp1; // temperary variables to store data
float total_optic, total_left, total_right; // total optical flow, optical flow on the right/left
// set initial values for optical flow
total_optic=0;
total_left=0;
total_right=0;

// Sort the feature x locations x_new, and horizontal optical flow component dx_t
for (int i=0; i<(results->flow_count-1);i++){
	for (int j=0;j<(results->flow_count-1-i);j++){
		if (new_x[j]>new_x[j+1]){
		   temp0=new_x[j+1];
		   new_x[j+1]=new_x[j];
		   new_x[j]=temp0;
		   temp1=dx_t[j+1];
		   dx_t[j+1]=dx_t[j];
		   dx_t[j]=temp1;
		}
	}	
}
	// next loop calculates total flow and discard a beam in the middle of the image 
	// The size of the beam should be fine tuned to a value that is desirable
	for(int i = 0; i < results->flow_count; i++){
		//beam
		if (new_x[i] > ((w/2) - w*0.05) && new_x[i]<(((w/2) + w*0.05) ){ 
			dx_t[i]=0;
			}
		//flow on the left side of the frame
		if (new_x[i] < ((w/2) - w*0.05)){
			distance_correction = (0.5*w - new_x[i]) / (0.5*w) //apply distance correction
			results->OFtotalL += -1*dx_t*distance_correction;
			}
		// flow on the right side of the frame
		if (new_x[i]>(((w/2) + w*0.05))){
			distance_correction = ( -1*(new_x[i] - w)) / (0.5*w)
			results->OFtotalR += dx_t*distance_correction;
			}
		}
	// Total optical flow
	results->OFtotal=results->OFtotalR+results->OFtotalL;
*/

//------------------------------------------------------------------


// ADD featureless zone algorithm...

/*


float dx_feature; // horizontal distance between two points
int loc_feature; // corresponding location

for(int i=0;i<(results->flow_count-1);i++){

    dx_feature=new_x[i+1]-new_x[i]; // distance between 2 consecutive feature x-coordinates
	if(dx_feature>results->OFFlesszone){
		results->OFFlessZone=dx_feature;  // Largest featureless zone of current frame
		loc_feature = dx_feature /2 + new_x[i];  // middle of featureless zone
		results->OFFlessZonePos=loc_feature;  // middle of largest featureless zone of c. frame
	}
}

*/


// ADD commands...


/* REMOVED_MAV
  // Average Filter
  OFfilter(&results->OFx, &results->OFy, OFx_trans, OFy_trans, results->flow_count, 1);

  // Velocity Computation
  if (info->agl < 0.01) {
    results->cam_h = 0.01;
  }
  else {
    results->cam_h = info->agl;
  }

  if (results->flow_count) {
    results->Velx = results->OFy * results->cam_h * results->FPS / Fy_ARdrone + 0.05;
    results->Vely = -results->OFx * results->cam_h * results->FPS / Fx_ARdrone - 0.1;
  } else {
    results->Velx = 0.0;
    results->Vely = 0.0;
  }
REMOVED_MAV */
  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************

  memcpy(visual_estimator.prev_frame, frame, w * h * 2);	//data at pointer new_frame.buf is copied to memory at pointer previous_frame
  memcpy(visual_estimator.prev_gray_frame, visual_estimator.gray_frame, w * h);	//data at pointer gray_frame is copied to memory at pointer previous_gray_frame

}
