#ifndef WHISKER_SIMULATOR_H
#define WHISKER_SIMULATOR_H

/* ros includes */
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>

/* amigo includes */
#include <tue/config/configuration.h>

/* stem tracking includes */
#include "loggingmacros.h"

#define     THIS_PACKAGE    "stem_tracker"
#define     THIS_NODE       "topsensor_locator"
#define     THIS_NODE_RATE  20

float min_val_right = 10.0;
float min_val_left = 10.0;

#endif // WHISKER_SIMULATOR_H
