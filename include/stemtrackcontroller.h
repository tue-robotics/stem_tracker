#ifndef STEMTRACKCONTROLLER_H
#define STEMTRACKCONTROLLER_H

#define     INFO_STREAM     ROS_INFO_STREAM

#include <ros/ros.h>
#include <vector>
#include <math.h>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "debugfunctions.h"

class RobotRepresentation;
class StemRepresentation;
class RobotStatus;

class StemTrackController
{
private:
    float m_max_z_dot;
    int m_update_rate;
    bool m_tilt_with_stem;
    bool m_debug_ik_solver;
    bool m_use_ik_velocity_solver_only;
    KDL::Vector m_setpoint_vector;
    KDL::Frame m_setpoint_frame;
    RobotRepresentation* m_p_robot_representation;
    StemRepresentation* m_p_stem_representation;
    RobotStatus* m_p_robot_status;
    KDL::JntArray m_joint_pos_refs;
    KDL::JntArray m_joint_vel_refs;
public:
    StemTrackController(RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status, StemRepresentation* p_stem_representation)
        : m_p_robot_representation(p_robot_representation), m_p_stem_representation(p_stem_representation), m_p_robot_status(p_robot_status), m_debug_ik_solver(false) {}
    void setMaxZvelocity(float max_z_dot);
    void setUpdateRate(int update_rate);
    void setUseInverseVelocitySolverOnly(bool use_ik_vel_only);
    void setTiltWithStem(bool tilt_with_stem);
    void updateCartSetpoint(std::vector<float> setpoint_xyz);
    void updateCartSetpoint(std::vector<float> gripper_xyz, std::vector<float> xy_err);
    void updateJointPosReferences();
    void updateJointVelReferences();
    void turnVelRefInPosRef();
    void setDebugIKsolver(bool debug_ik_solver);
    KDL::Vector getCartSetpointKDLVect();
    KDL::Frame getCartSetpointKDLFrame();
    KDL::JntArray getJointPosRefs();
    KDL::JntArray getJointVelRefs();
    ~StemTrackController();

};

#endif // STEMTRACKCONTROLLER_H
