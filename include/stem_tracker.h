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
#include "debugfunctions.h"
#include "whiskerinterpreter.h"
#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "visualizationinterface.h"
#include "robotinterface.h"
#include "stemtrackcontroller.h"
#include "stemtrackmonitor.h"
#include "configurer.h"

#define     INFO_STREAM     ROS_INFO_STREAM
#define     THIS_PACKAGE    "stem_tracker"

#endif // STEM_TRACKER_H
