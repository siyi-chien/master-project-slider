/******************************************************************************
-Author: Ke Wang & Zhonghe Jiang (Harry)
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2019
******************************************************************************/

#ifndef DEFINE_H_
#define DEFINE_H_

#include <cmath>

#define LOOP_RATE 1000 // loop rate for the controller
#define GRAVITY 9.81
#define PI 3.14159

enum Support { L = -1, D, R, S, F}; 

// Initial pelvis height
const double pelvis_height = 0.70;

// CoM height
const double com_height = 0.7;

// Height of the end-effector
const double foot_height = 0.045;

// Peak of the swing foot
const double step_height = 0.095;

// Step width
const double step_width = 0.28;

// Step time
const double step_time = 0.7;

// Time required for the initial sway
const double initial_sway_time = 0.7;

// Time delay before updating robot configuration from Gazebo data
const double update_delay = 100;

const double k_LIPM = sqrt(com_height/GRAVITY);


#endif /* DEFINE_H_ */