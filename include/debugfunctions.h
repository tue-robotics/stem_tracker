#ifndef DEBUGFUNCTIONS_H
#define DEBUGFUNCTIONS_H

#include <kdl/frames.hpp>
#include <ros/ros.h>

#define INFO_STREAM     ROS_INFO_STREAM

void printKDLframe(KDL::Frame kdl_frame);
void printXYZvector(std::vector<float> vect);

#endif // DEBUGFUNCTIONS_H
