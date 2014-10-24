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

/* amigo includes */
#include <profiling/StatsPublisher.h>
#include <tue/config/configuration.h>

/* stem tracking includes */

#include "whiskerinterpreter.h"
#include "stemrepresentation.h"
#include "robotconfig.h"
#include "robotstatus.h"

/* configure */

#define DEBUG                           true                            // if true additional information will be printed
#define INFO_STREAM                     ROS_INFO_STREAM

#define UPDATE_RATE                     100                             // spin rate of this node, in hz

#define USE_LEFTARM                     true                            // use left arm if true, else use right arm
#define ROBOT_DESCRIPTION_ROSPARAM      "/amigo/robot_description"      // amigo urdf model gets loaded in this rosparam

#define STEM_R                          0.05
#define STEM_G                          0.65
#define STEM_B                          0.35
#define STEM_THICKNESS                  0.03                            // thickness in meters

float X[] = {  0.3,     0.35,   0.3,    0.35,   0.3,    0.25    };
float Y[] = {  0.3,     0.35,   0.35,   0.4,    0.55,   0.6     };
float Z[] = {  0.4,     0.6,    0.9,    1.2,    1.4,    1.6     };
std::vector<float> stemNodesX(X, X + sizeof(X) / sizeof(*X) );
std::vector<float> stemNodesY(Y, Y + sizeof(Y) / sizeof(*Y) );
std::vector<float> stemNodesZ(Z, Z + sizeof(Z) / sizeof(*Z) );

std::string ROOT_LINK;

#define LEFT_END_LINK   "grippoint_left"
#define RIGHT_END_LINK  "grippoint_right"


/* initialize */

bool initializing = true;
int state = 0;
int i, up = 1;
ros::Publisher visualization_publisher;
ros::Publisher arm_reference_publisher;
ros::Subscriber arm_measurements_subscriber;
ros::Subscriber torso_measurements_subscriber;

StatsPublisher sp;


#endif // STEM_TRACKER_H
