#ifndef STEMTRACKCONTROLLER_H
#define STEMTRACKCONTROLLER_H

#define     INFO_STREAM     ROS_INFO_STREAM

#include <ros/ros.h>
#include <vector>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "robotconfig.h"
#include "robotstatus.h"

class StemTrackController
{
private:
    float m_max_z_dot;
    int m_update_rate;
    KDL::Vector m_setpoint;
    RobotConfig* m_p_robot_config;
    RobotStatus* m_p_robot_status;
    KDL::JntArray m_joint_refs;
public:
    StemTrackController(float max_z_dot, int update_rate, RobotConfig* p_robot_config, RobotStatus* p_robot_status);
    void updateCartSetpoint(std::vector<float> gripper_xyz, std::vector<float> xy_err,  int up);
    void updateJointReferences();
    KDL::Vector getCartSetpointKDLVect();
    KDL::JntArray getJointRefs();
    ~StemTrackController();
};

#endif // STEMTRACKCONTROLLER_H
