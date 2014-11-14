#include "stemtrackcontroller.h"

StemTrackController::StemTrackController(RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status, StemRepresentation* p_stem_representation)
{
    m_p_robot_representation = p_robot_representation;
    m_p_stem_representation = p_stem_representation;
    m_p_robot_status = p_robot_status;
    m_debug_ik_solver = false;
}

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

void StemTrackController::updateCartSetpoint(std::vector<float> setpoint_xyz)
{
    if(setpoint_xyz.size() != 3)
        INFO_STREAM("unexpected vector length in update cart setpoint, setpoint_xyz.size() = " << setpoint_xyz.size() );

    m_setpoint_vector = KDL::Vector( setpoint_xyz[0], setpoint_xyz[1], setpoint_xyz[2]);
    m_setpoint_frame = KDL::Frame( KDL::Rotation::Identity(), m_setpoint_vector);

    // todo: clever pose/rotation for stem grasping

    return;
}

void StemTrackController::updateCartSetpoint(std::vector<float> gripper_xyz, std::vector<float> xy_err)
{
    if(gripper_xyz.size() != 3 || xy_err.size() != 2)
        INFO_STREAM("unexpected vector length in update cart setpoint, grippper_xyz.size() = " << gripper_xyz.size() << " xy_err.size() = " << xy_err.size() );

    m_setpoint_vector = KDL::Vector( gripper_xyz[0] - xy_err[0], gripper_xyz[1] - xy_err[1], gripper_xyz[2] + m_max_z_dot / (double) m_update_rate );

    m_setpoint_frame = KDL::Frame( KDL::Rotation::Identity(), m_setpoint_vector);

    KDL::Rotation gripper_rotation = KDL::Rotation::Identity();

    // assumption: 'neutral' gripping pose is not rotated wrt base frame

    if(m_tilt_with_stem)
    {
        gripper_rotation.DoRotZ(-0.3);
//        m_p_stem_representation->getCurrentDerivative();
    }

    m_setpoint_frame = KDL::Frame( gripper_rotation, m_setpoint_vector);

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

void StemTrackController::updateJointReferences()
{
    boost::shared_ptr<KDL::ChainFkSolverPos> fksolver_;
    boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverPos> ik_solver_;

    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(m_p_robot_representation->getKinematicChain()));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(m_p_robot_representation->getKinematicChain()));
    ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(m_p_robot_representation->getKinematicChain(), m_p_robot_representation->getJointMinima(), m_p_robot_representation->getJointMaxima(), *fksolver_, *ik_vel_solver_, 100) );


    KDL::Frame f_in(m_p_robot_status->getGripperKDLframe().M, m_setpoint_vector );

    int status = ik_solver_->CartToJnt(m_p_robot_representation->getJointSeeds(), m_setpoint_frame, m_joint_refs);
    if(m_debug_ik_solver)
        INFO_STREAM("status ik_solver: " << status);

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
