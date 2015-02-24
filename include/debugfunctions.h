#ifndef DEBUGFUNCTIONS_H
#define DEBUGFUNCTIONS_H

#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jntarray.hpp>
#include <vector>

void printKDLframe(KDL::Frame kdl_frame);
void printVector(std::vector<float> vect);
void printVector(std::vector<bool> vect);
void printKDLVelframe(KDL::FrameVel kdl_vel_frame);
void printKDLJntArray(KDL::JntArray kdl_array);


#endif // DEBUGFUNCTIONS_H
