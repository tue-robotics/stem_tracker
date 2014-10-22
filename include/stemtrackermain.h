#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H


/* c++ includes */
#include <vector>
#include <map>
#include <iostream>
#include <cmath>

#include <boost/shared_ptr.hpp>

/* ros includes */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <ros/package.h> // used in config loading

/* kdl includes */
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

//#include <kdl/treejnttojacsolver.hpp>

/* amigo includes */
#include <profiling/StatsPublisher.h>
#include <tue/config/configuration.h>



#endif // STEM_TRACKER_H
