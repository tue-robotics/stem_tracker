#include "stemtrackcontroller.h"

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "debugfunctions.h"
#include "robotrepresentation.h"
#include "stemrepresentation.h"
#include "robotstatus.h"
#include "loggingmacros.h"

void StemTrackController::setDebugIKsolver(bool debug_ik_solver)
{
    m_debug_ik_solver = debug_ik_solver;
}

void StemTrackController::setMaxZvelocity(float max_z_dot)
{
    m_max_z_dot = max_z_dot;
}

void StemTrackController::setUpdateRate(int update_rate)
{
    m_update_rate = update_rate;
}

void StemTrackController::setTiltWithStem(bool tilt_with_stem)
{
    m_tilt_with_stem = tilt_with_stem;
}

void StemTrackController::updateCartSetpoint(const std::vector<float> setpoint_xyz)
{
    if(setpoint_xyz.size() != 3)
        ERROR_STREAM("Unexpected vector length in update cart setpoint, setpoint_xyz.size() = " << setpoint_xyz.size() << ".");

    m_setpoint_vector = KDL::Vector( setpoint_xyz[0], setpoint_xyz[1], setpoint_xyz[2]);

    KDL::Rotation gripper_rotation = KDL::Rotation::Identity();

    if(m_tilt_with_stem)
    {
        std::vector<float> stem_tangent = m_p_stem_representation->getCurrentTangent();

        if(stem_tangent.size() == 3)
        {
            float len = sqrt(stem_tangent[0] * stem_tangent[0] + stem_tangent[1] * stem_tangent[1] + stem_tangent[2] * stem_tangent[2]);
            if(len > 0.0)
            {
                gripper_rotation.DoRotX( asin(-stem_tangent[1]/len) );
                gripper_rotation.DoRotY( asin(stem_tangent[0]/len) );
            }
        }
    }

    m_setpoint_frame = KDL::Frame( gripper_rotation, m_setpoint_vector);

    return;
}

void StemTrackController::setPointMoveForward(const std::vector<float> gripper_xyz, const float dist, const float z)
{
    if(!gripper_xyz.size() == 3)
    {
        WARNING_STREAM("In setPointMoveForward gripper_xyz contains " << gripper_xyz.size() << " elements. I need xyz.");
        return;
    }

    m_setpoint_vector = KDL::Vector(gripper_xyz[0]+dist, gripper_xyz[1], z);
    m_setpoint_frame = KDL::Frame( KDL::Rotation::Identity(), m_setpoint_vector);
    return;
}

void StemTrackController::updateCartSetpoint(const std::vector<float> gripper_xyz, const std::vector<float> xyz_err)
{
    if(gripper_xyz.size() != 3 || xyz_err.size() != 3)
        ERROR_STREAM("Unexpected vector length in update cart setpoint, grippper_xyz.size() = " << gripper_xyz.size() << " xyz_err.size() = " << xyz_err.size() << ".");

    std::vector<float> setpoint_vector;
    setpoint_vector.assign(3,0.0);
    setpoint_vector[0] = gripper_xyz[0] - xyz_err[0];
    setpoint_vector[1] =  gripper_xyz[1] - xyz_err[1];
    setpoint_vector[2] = gripper_xyz[2] + m_max_z_dot / (double) m_update_rate;

    updateCartSetpoint(setpoint_vector);

    return;
}

KDL::Vector StemTrackController::getCartSetpointKDLVect()
{
    return m_setpoint_vector;
}

KDL::Frame StemTrackController::getCartSetpointKDLFrame()
{
    return m_setpoint_frame;
}

void StemTrackController::setUseInverseVelocitySolverOnly(bool use_ik_vel_only)
{
    m_use_ik_velocity_solver_only = use_ik_vel_only;
}

void StemTrackController::updateJointPosReferences()
{
    if(m_use_ik_velocity_solver_only){
        updateJointVelReferences();
        turnVelRefInPosRef();
    }
    else
    {
        boost::shared_ptr<KDL::ChainFkSolverPos> fksolver_;
        boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
        boost::shared_ptr<KDL::ChainIkSolverPos> ik_solver_;

        fksolver_.reset(new KDL::ChainFkSolverPos_recursive(m_p_robot_representation->getKinematicChain()));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(m_p_robot_representation->getKinematicChain()));
        ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(m_p_robot_representation->getKinematicChain(), m_p_robot_representation->getJointMinima(), m_p_robot_representation->getJointMaxima(), *fksolver_, *ik_vel_solver_, 100) );


        KDL::Frame f_in(m_p_robot_status->getGripperKDLframe().M, m_setpoint_vector );

        int status = ik_solver_->CartToJnt(m_p_robot_representation->getJointSeeds(), m_setpoint_frame, m_joint_pos_refs);
        if(m_debug_ik_solver)
            INFO_STREAM("Status ik_solver: " << status);
    }
    return;
}

void StemTrackController::updateJointVelReferences()
{
    KDL::Vector vel = m_setpoint_vector - m_p_robot_status->getGripperKDLframe().p;
    KDL::Vector rot = m_setpoint_frame.M.GetRot();
    KDL::Twist twist = KDL::Twist(vel, rot);

    boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(m_p_robot_representation->getKinematicChain()));

    m_joint_vel_refs.resize(m_p_robot_status->getNjointsMonitoring());
    ik_vel_solver_->CartToJnt(m_p_robot_status->getJointStatus(), twist, m_joint_vel_refs);
}

void StemTrackController::turnVelRefInPosRef()
{
    m_joint_pos_refs.resize(m_joint_vel_refs.rows());
    for(int i=0; i<m_joint_vel_refs.rows(); ++i)
    {
        double tmp = m_p_robot_status->getJointStatus()(i) + m_joint_vel_refs(i) * 1.0 / (double) m_update_rate;
        if(tmp>m_p_robot_representation->getJointMaxima()(i))
            m_joint_pos_refs(i) = m_p_robot_representation->getJointMaxima()(i);
        else if (tmp<m_p_robot_representation->getJointMinima()(i))
            m_joint_pos_refs(i) = m_p_robot_representation->getJointMinima()(i);
        else
            m_joint_pos_refs(i) = tmp;
    }
}

KDL::JntArray StemTrackController::getJointPosRefs()
{
    return m_joint_pos_refs;
}

KDL::JntArray StemTrackController::getJointVelRefs()
{
    return m_joint_vel_refs;
}


StemTrackController::~StemTrackController()
{
    //destructor
}
