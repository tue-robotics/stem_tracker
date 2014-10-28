#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H

/* c++ includes */
#include <vector>
#include <iostream>

/* ros includes */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <ros/package.h> // used in config loading

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>


/* amigo includes */
#include <profiling/StatsPublisher.h>
#include <tue/config/configuration.h>

/* stem tracking includes */

#include "whiskerinterpreter.h"
#include "stemrepresentation.h"
#include "robotconfig.h"
#include "robotstatus.h"

#define     INFO_STREAM     ROS_INFO_STREAM
#define     THIS_PACKAGE    "stem_tracker"

/* init configurable params */

bool DEBUG;
bool DEBUG_IK;
int UPDATE_RATE;
std::string BASE_FRAME;

std::string ROBOT_NAME;
std::string ROOT_LINK;
bool USE_LEFTARM;
std::string ROBOT_DESCRIPTION_ROSPARAM;
std::string LEFT_END_LINK;
std::string RIGHT_END_LINK;
double UP_TO_DATE_THRESHOLD;

int N_WHISKERS;
float GRIPPER_DIAMETER;
float WHISKER_LENGTH;

float STEM_THICKNESS;
float STEM_RGB[3];
std::vector<float> stemNodesX;
std::vector<float> stemNodesY;
std::vector<float> stemNodesZ;


#endif // STEM_TRACKER_H
