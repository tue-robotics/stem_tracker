#ifndef STEMTRACKCONTROLLER_H
#define STEMTRACKCONTROLLER_H

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

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

    inline void setDebugIKsolver(bool debug_ik_solver) { m_debug_ik_solver = debug_ik_solver; }
    inline void setMaxZvelocity(float max_z_dot) { m_max_z_dot = max_z_dot; }
    inline void setUpdateRate(int update_rate) { m_update_rate = update_rate; }
    inline void setTiltWithStem(bool tilt_with_stem) { m_tilt_with_stem = tilt_with_stem; }
    inline void setUseInverseVelocitySolverOnly(bool use_ik_vel_only) { m_use_ik_velocity_solver_only = use_ik_vel_only; }

    void setCartSetpoint(const std::vector<float> setpoint_xyz);
    void setCartSetpoint(const std::vector<float> gripper_xyz, const std::vector<float> xyz_err);
    void updateJointPosReferences();
    void updateJointVelReferences();
    void turnVelRefInPosRef();
    void setPointMoveForward(const std::vector<float> gripper_xyz, const float dist, const float z);

    inline KDL::JntArray getJointPosRefs() { return m_joint_pos_refs; }
    KDL::JntArray getJointVelRefs() { return m_joint_vel_refs; }
    inline const KDL::Vector getCartSetpointKDLVect() { return m_setpoint_vector; }
    inline const KDL::Frame getCartSetpointKDLFrame() { return m_setpoint_frame; }

    ~StemTrackController();

};

#endif // STEMTRACKCONTROLLER_H
