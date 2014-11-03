#include "stemtrackcontroller.h"

StemTrackController::StemTrackController(float max_z_dot, int update_rate, RobotConfig* p_robot_config, RobotStatus* p_robot_status)
{
    m_max_z_dot = max_z_dot;
    m_update_rate = update_rate;
    m_p_robot_config = p_robot_config;
    m_p_robot_status = p_robot_status;
}

void StemTrackController::updateCartSetpoint(std::vector<float> gripper_xyz, std::vector<float> xy_err,  int up)
{
    if(gripper_xyz.size() != 3 || xy_err.size() != 2)
        INFO_STREAM("unexpected vector length in update cart setpoint, grippper_xyz.size() = " << gripper_xyz.size() << " xy_err.size() = " << xy_err.size() );

    m_setpoint = KDL::Vector( (double) gripper_xyz[0] - (double) xy_err[0], (double) gripper_xyz[1] - (double) xy_err[1], (double) gripper_xyz[2] + (double) m_max_z_dot / (double) m_update_rate * (double) up );

    return;
}

KDL::Vector StemTrackController::getCartSetpointKDLVect()
{
    return m_setpoint;
}

void StemTrackController::updateJointReferences()
{
    boost::shared_ptr<KDL::ChainFkSolverPos> fksolver_;
    boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverPos> ik_solver_;

    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(m_p_robot_config->getKinematicChain()));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(m_p_robot_config->getKinematicChain()));
    ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(m_p_robot_config->getKinematicChain(), m_p_robot_config->getJointMinima(), m_p_robot_config->getJointMaxima(), *fksolver_, *ik_vel_solver_, 100) );


    KDL::Frame f_in(m_p_robot_status->getGripperKDLframe().M, m_setpoint );

    int status = ik_solver_->CartToJnt(m_p_robot_config->getJointSeeds(), f_in, m_joint_refs);

    return;
}

KDL::JntArray StemTrackController::getJointRefs()
{
    return m_joint_refs;
}

StemTrackController::~StemTrackController()
{
    //destructor
}
