#ifndef STEMTRACKCONTROLLER_H
#define STEMTRACKCONTROLLER_H

#define     INFO_STREAM     ROS_INFO_STREAM

#include <ros/ros.h>
#include <vector>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "robotrepresentation.h"
#include "robotstatus.h"

class StemTrackController
{
private:
    float m_max_z_dot;
    int m_update_rate;
    KDL::Vector m_setpoint_vector;
    KDL::Frame m_setpoint_frame;
    RobotRepresentation* m_p_robot_representation;
    RobotStatus* m_p_robot_status;
    KDL::JntArray m_joint_refs;
public:
    StemTrackController(RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status);
    void setMaxZvelocity(float max_z_dot);
    void setUpdateRate(int update_rate);
    void updateCartSetpoint(std::vector<float> gripper_xyz, std::vector<float> xy_err,  int up);
    void updateJointReferences();
    KDL::Vector getCartSetpointKDLVect();
    KDL::Frame getCartSetpointKDLFrame();
    KDL::JntArray getJointRefs();
    ~StemTrackController();
};

#endif // STEMTRACKCONTROLLER_H
