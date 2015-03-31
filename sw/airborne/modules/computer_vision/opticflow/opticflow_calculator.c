/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/opticflow/opticflow_calculator.c
 * @brief Estimate velocity from optic flow.
 *
 * Using images from a vertical camera and IMU sensor data.
 */

#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "state.h"

#include <math.h>

// Own Header
#include "opticflow_calculator.h"

// Computer Vision
#include "lib/vision/image.h"
#include "lib/vision/lucas_kanade.h"
#include "lib/vision/fast_rosten.h"

// ARDrone Vertical Camera Parameters
#define FOV_H 0.67020643276		// Tobias: change to Frontal camera parameter
#define FOV_W 1.60		// Assumed field of view angle of forntal camera 
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053

/* Functions only used here */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);

/**
 * Initialize the opticflow calculator
 * @param[out] *opticflow The new optical flow calculator
 * @param[in] *w The image width
 * @param[in] *h The image height
 */
void opticflow_calc_init(struct opticflow_t *opticflow, uint16_t w, uint16_t h)
{

  /* Create the image buffers */
  image_create(&opticflow->img_gray, w, h, IMAGE_GRAYSCALE);		// Tobias: (output image struct, input width, input height, input image type)
  image_create(&opticflow->prev_img_gray, w, h, IMAGE_GRAYSCALE);

  /* Set the previous values */
  opticflow->got_first_img = FALSE;
  opticflow->prev_phi = 0.0;
  opticflow->prev_psi = 0.0;		// Tobias: added prev_psi for derotation of frontal camera opticflow
  opticflow->prev_theta = 0.0;
}


/**
 * Run the optical flow on a new image frame
 * @param[in] *opticflow The opticalflow structure that keeps track of previous images
 * @param[in] *state The state of the drone
 * @param[in] *img The image frame to calculate the optical flow from
 * @param[out] *result The optical flow result
 */
void opticflow_calc_frame(struct opticflow_t *opticflow, struct opticflow_state_t *att_state, struct image_t *img, struct opticflow_result_t *result)
{
  // Update FPS for information
  result->fps = 1 / (timeval_diff(&opticflow->prev_timestamp, &img->ts) / 1000.);
  memcpy(&opticflow->prev_timestamp, &img->ts, sizeof(struct timeval));

  // Convert image to grayscale
  image_to_grayscale(img, &opticflow->img_gray);	// Tobias: (input image struct (img->buf), output image struct (img->buf) )

  // Copy to previous image if not set
  if (!opticflow->got_first_img) {
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);	// Tobias: (input image struct, output image struct )
    opticflow->got_first_img = TRUE;
  }

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection
  static uint8_t threshold = 20;
//  threshold = 20;	// Tobias: Uncomment this to turn of adaptive threshold
  struct point_t *corners = fast9_detect(img, threshold, 10, 20, 20, &result->corner_cnt);
// printf("Fast corners Corners = %i  threshold = %i	\n",result->corner_cnt,threshold);

  // Adaptive threshold
  if(result->corner_cnt < 40 && threshold > 5)
    threshold--;
  else if(result->corner_cnt > 50 && threshold < 60)
    threshold++;

  // Check if we found some corners to track
  if(result->corner_cnt < 1) {
    free(corners);
    image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
    return;
  }
  

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************

#define MAX_TRACK_CORNERS 100
#define HALF_WINDOW_SIZE 5
#define SUBPIXEL_FACTOR 10
#define MAX_ITERATIONS 10
#define THRESHOLD_VEC 2
  // Execute a Lucas Kanade optical flow
  result->tracked_cnt = result->corner_cnt;
  struct flow_t *vectors = opticFlowLK(&opticflow->img_gray, &opticflow->prev_img_gray, corners, &result->tracked_cnt,
    HALF_WINDOW_SIZE, SUBPIXEL_FACTOR, MAX_ITERATIONS, THRESHOLD_VEC, MAX_TRACK_CORNERS);


// ***************** Start Display results x,y,dx,dy for MATLAB *****************
/*
printf("A = [");
for (int i = 0; i < result->corner_cnt;i++){
printf("%i, %i, %i, %i;",corners[i].x,corners[i].y,vectors[i].flow_x,vectors[i].flow_y);
}
printf("]	\n");	// replace ;] with ]
printf("x = A(:,1);y= A(:,2);dx = A(:,3);dy = A(:,4);	\n");
printf("figure(1);	\n");
printf("quiver(x,y,dx,dy);	\n");
printf("figure(2);	\n");
printf("plot(x,y,'*');	\n");
printf("pause;	\n");  */
// ***************** End Display results x,y,dx,dy for MATLAB *****************


// ***************** Start Display optic flow results ***************** 
/*
for (int i = 0; i < result->corner_cnt;i++){
printf("dx=%i, ",vectors[i].flow_x);
}
printf("	\n"); */
// ***************** End Display optic flow results ***************** 


// ***************** Start sorting corners & optic flow vectos ***************** 
// Note: Use Insertion Sort algorithm, 
  uint16_t j;
  for (uint16_t i = 1; i < result->tracked_cnt; ++i)
  {
    struct flow_t idx_vectors;		// Tobias: Optic flow vectors flow_x,flow_y
    struct point_t idx_corners;		// Tobias: Corners x,y
  
    idx_vectors = vectors[i];		// Tobias: Temporary struct
    idx_corners = corners[i];
  
    for (j = i; j > 0 && corners[j-1].x > idx_corners.x; j--)	// Tobias: Sort based on x coordinate of the corners
    {
      vectors[j] = vectors[j-1];
      corners[j] = corners[j-1];
    }

    vectors[j] = idx_vectors;
    corners[j] = idx_corners;
  } 
// ***************** End sorting corners & optic flow vectos ***************** 


// ***************** Start sorting Left, Right and center position of optic flow vectos ***************** 
/*result->tot_of_left = 0;
result->tot_of_right = 0;
result->tot_of = 0;

for (int i = 0; i < result->tracked_cnt; i++){
    if (corners[i].x < 0.45 * img->w){
        result->tot_of_left = result->tot_of_left - (img->w * vectors[i].flow_x  /abs(img->w/2 - corners[i].x)) ;	// Tobias: total optic flow left
        result->tot_of = result->tot_of + (img->w * abs(vectors[i].flow_x)/abs(img->w/2 - corners[i].x)) ;	// Tobias: Total optic flow
    }
    else if(corners[i].x > 0.55 * img->w){
    	result->tot_of_right = result->tot_of_right + (img->w * vectors[i].flow_x/abs(corners[i].x-img->w/2));	// Tobias: total optic flow right
    	result->tot_of = result->tot_of + (img->w * abs(vectors[i].flow_x)/abs(corners[i].x-img->w/2));	// Tobias: Total optic flow
    }
}*/
// ***************** End sorting Left, Right and center position of optic flow vectos ***************** 

//printf("Not derotated: left %0.3f	\n",result->tot_of_left);
//printf("Not derotated: right %0.3f \n",result->tot_of_right);



// ***************** Start derotation of optic flow ***************** 
  float diff_flow_x_psi = (att_state->psi - opticflow->prev_psi) * img->w / FOV_W;	// Tobias: only for yaw corrected
for (int i = 0; i < result->tracked_cnt; i++)
  {
   vectors[i].flow_x = vectors[i].flow_x + diff_flow_x_psi * SUBPIXEL_FACTOR /1.2;
   vectors[i].flow_y = 0;
  }

  opticflow->prev_phi = att_state->phi;
  opticflow->prev_psi = att_state->psi;		// Tobias: added previous psi = current psi
  opticflow->prev_theta = att_state->theta;

// ***************** End derotation of optic flow ***************** 


// ***************** Start sorting Left, Right and center position of optic flow vectos ***************** 
result->tot_of_left = 0;
result->tot_of_right = 0;
result->tot_of = 0;

for (int i = 0; i < result->tracked_cnt; i++){
    if (corners[i].x < 0.45 * img->w){
        result->tot_of_left = result->tot_of_left - (img->w * vectors[i].flow_x  /abs(img->w/2 - corners[i].x)) ;	// Tobias: total optic flow left
        result->tot_of = result->tot_of + (img->w * abs(vectors[i].flow_x)/abs(img->w/2 - corners[i].x)) ;	// Tobias: Total optic flow
    }
    else if(corners[i].x > 0.55 * img->w){
    	result->tot_of_right = result->tot_of_right + (img->w * vectors[i].flow_x/abs(corners[i].x-img->w/2));	// Tobias: total optic flow right
    	result->tot_of = result->tot_of + (img->w * abs(vectors[i].flow_x)/abs(corners[i].x-img->w/2));	// Tobias: Total optic flow
    }
}
// ***************** End sorting Left, Right and center position of optic flow vectos ***************** 

// ***************** Start Display results ***************** 
//printf("psi_f = %0.3f	\n",stateGetNedToBodyEulers_f()->phi);
//printf("psi = %0.3f, dpsi = %0.3f	\n",att_state->psi,att_state->psi - opticflow->prev_psi);
//printf("phi = %0.3f, dphi = %0.3f	\n",att_state->phi,att_state->phi - opticflow->prev_phi);
//printf("   derotated: left  %0.3f, yaw_induced_dx = %0.3f	\n",result->tot_of_left,diff_flow_x_psi);
//printf("   derotated: right %0.3f, yaw_induced_dx = %0.3f	\n",result->tot_of_right,diff_flow_x_psi);
//printf("   derotated: total %0.3f	\n",result->tot_of);
// ***************** End Display results ***************** 

uint16_t corner_separation = 0;		// horizontal distance between two points
uint16_t loc_feature = 0;		// corresponding location

result->of_featurelesszone = 0;		// (Re)set value to size of featureless zone
result->of_featurelesszone_pos = 1;		// (Re)set value to size of featureless zone

for(int i = 0; i < (result->tracked_cnt - 1);i++){

  corner_separation = corners[i+1].x - corners[i].x;		// Distance between 2 consecutive corners
  if(corner_separation > result->of_featurelesszone){
    result->of_featurelesszone = corner_separation;		// Largest featureless zone of current frame
    loc_feature = corner_separation /2 + corners[i].x;		// Middle of featureless zone
    result->of_featurelesszone_pos = loc_feature;		// Store middle of featureless zone in result struct
  }
}




  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************
  free(corners);
  free(vectors);
  image_copy(&opticflow->img_gray, &opticflow->prev_img_gray);
}



/**
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 * @return The difference in milliseconds
 */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
  uint32_t msec;
  msec=(finishtime->tv_sec-starttime->tv_sec)*1000;
  msec+=(finishtime->tv_usec-starttime->tv_usec)/1000;
  return msec;
}


