#ifndef DEBUGFUNCTIONS_H
#define DEBUGFUNCTIONS_H

#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jntarray.hpp>
#include <vector>

#include "loggingmacros.h"

void printKDLframe(KDL::Frame kdl_frame);
void printKDLVelframe(KDL::FrameVel kdl_vel_frame);
void printKDLJntArray(KDL::JntArray kdl_array);

template<class TYPE>
void printVector(const TYPE& vect)
{
    if(vect.size() == 0)
    {
        INFO_STREAM("vector empty");
        return;
    }

    std::stringstream tmp;
    for(uint i = 0; i < vect.size(); ++i)
        tmp << "vect[" << i << "] = " << vect.at(i) << "  ";

    INFO_STREAM(tmp.str());
    return;
}

#endif // DEBUGFUNCTIONS_H
