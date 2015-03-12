#include "stemtrackcontroller.h"

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "debugfunctions.h"
#include "robotrepresentation.h"
#include "stemrepresentation.h"
#include "robotstatus.h"
#include "loggingmacros.h"

void StemTrackController::updateCartSetpoint(const std::vector<float>& gripper_pos_err)
{
    /* Check position error from gripper (touch), if no touch error than move upward, else
       move in direction to undo touch. gripper_pos_error is defined in the frame of the gripper */

    for(uint i = 0; i < 3; ++i)
    {
        if(gripper_pos_err[i] > 0.0)
        {
            std::vector<float> setpoint;
            setpoint.push_back(m_p_robot_status->gripperFrameVectorToBaseFrameVector(gripper_pos_err)[0]);
            setpoint.push_back(m_p_robot_status->gripperFrameVectorToBaseFrameVector(gripper_pos_err)[1]);
            if(m_touch_started_at_xyz.size() != 3)
            {
                ERROR_STREAM("Touched but xyz not stored in m_touch_started_at_xyz!");
                return;
            }
            setpoint.push_back(m_touch_started_at_xyz[2]);
            setCartSetpoint(setpoint);
            return;
        }
    }
    setPointMoveUp();
    return;
}

void StemTrackController::setPointMoveUp()
{
    /*todo: add case for tilt_with stem
            take current stem tangent into account when moving up */

    float move_up_ref = m_move_up_ref;
    if(m_p_robot_status->amigoTorsoIsAtMax())  //ugly implementation, no amigo specific stuff here
    {
        move_up_ref = m_setpoint_multiplication_at_max_torso*m_move_up_ref;
    }
    m_setpoint_vector = KDL::Vector(m_p_robot_status->getGripperXYZ()[0], m_p_robot_status->getGripperXYZ()[1],
                                    m_p_robot_status->getGripperXYZ()[2]+move_up_ref);

    KDL::Rotation gripper_rotation = getDesiredGripperPose();
    m_setpoint_frame = KDL::Frame( gripper_rotation, m_setpoint_vector);

    return;
}

std::vector<float> StemTrackController::getCartSetpointXYZ()
{
    std::vector<float> xyz;
    xyz.push_back(m_setpoint_vector.x());
    xyz.push_back(m_setpoint_vector.y());
    xyz.push_back(m_setpoint_vector.z());
    return xyz;
}

KDL::Rotation StemTrackController::getDesiredGripperPose()
{
    float roll = 0.0, pitch = 0.0, yaw = 0.0;

    if(m_tilt_with_stem)
    {
        std::vector<float> stem_tangent = m_p_stem_representation->getTangent();

        if(m_p_stem_representation->getTangent().size() == 3)
        {
            float len = sqrt(stem_tangent[0] * stem_tangent[0] + stem_tangent[1] * stem_tangent[1] + stem_tangent[2] * stem_tangent[2]);
            if(len > 0.0)
            {
                roll = asin(-stem_tangent[1]/len);
                pitch = asin(stem_tangent[0]/len);
                INFO_STREAM("roll " << roll << " pitch: " << pitch);
            }
        }
    }

    return KDL::Rotation::RPY(roll, pitch, yaw);
}

std::vector< std::vector<float> > StemTrackController::getDesiredGripperPoseVectors()
{
    std::vector< std::vector<float> > xyz_xyz_xyz;

    KDL::Vector x = KDL::Vector(0.2,0.0,0.0);
    KDL::Vector y = KDL::Vector(0.0,0.2,0.0);
    KDL::Vector z = KDL::Vector(0.0,0.0,0.2);

    x = getDesiredGripperPose() * x;
    y = getDesiredGripperPose() * y;
    z = getDesiredGripperPose() * z;

    std::vector<float> tmp;
    tmp.push_back(x.x());
    tmp.push_back(x.y());
    tmp.push_back(x.z());
    xyz_xyz_xyz.push_back(tmp);

    tmp.clear();
    tmp.push_back(y.x());
    tmp.push_back(y.y());
    tmp.push_back(y.z());
    xyz_xyz_xyz.push_back(tmp);

    tmp.clear();
    tmp.push_back(z.x());
    tmp.push_back(z.y());
    tmp.push_back(z.z());
    xyz_xyz_xyz.push_back(tmp);

    return xyz_xyz_xyz;
}

void StemTrackController::setCartSetpoint(const std::vector<float> setpoint_xyz)
{
    /* sets setpoint for the gripper (defined in the baseframe) to setpoint_xyz */

    if(setpoint_xyz.size() != 3)
        ERROR_STREAM("Unexpected vector length in update cart setpoint, setpoint_xyz.size() = " << setpoint_xyz.size() << ".");

    m_setpoint_vector = KDL::Vector( setpoint_xyz[0], setpoint_xyz[1], setpoint_xyz[2]);

    KDL::Rotation gripper_rotation = getDesiredGripperPose();

    m_setpoint_frame = KDL::Frame( gripper_rotation, m_setpoint_vector);

    return;
}

void StemTrackController::setPointMoveForward(const std::vector<float> gripper_xyz, const float z)
{
    if(!gripper_xyz.size() == 3)
    {
        WARNING_STREAM("In setPointMoveForward gripper_xyz contains " << gripper_xyz.size() << " elements. I need xyz.");
        return;
    }

    m_setpoint_vector = KDL::Vector(gripper_xyz[0]+m_straight_forward_ref, gripper_xyz[1], z);
    m_setpoint_frame = KDL::Frame( KDL::Rotation::Identity(), m_setpoint_vector);
    return;
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
        ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(m_p_robot_representation->getKinematicChain(), m_p_robot_representation->getJointMinima(),
                                                         m_p_robot_representation->getJointMaxima(), *fksolver_, *ik_vel_solver_, 100) );


        //        KDL::Frame f_in(m_p_robot_status->getGripperKDLframe().M, m_setpoint_vector );

        int status = ik_solver_->CartToJnt(m_p_robot_representation->getJointSeeds(), m_setpoint_frame, m_joint_pos_refs);
        if(m_debug_ik_solver)
            INFO_STREAM("Status ik_solver: " << status);
    }

    if(m_print_ref_vs_current)
    {
        INFO_STREAM("refs:");
        printKDLJntArray(m_joint_pos_refs);
        INFO_STREAM("current:");
        printKDLJntArray(m_p_robot_status->getJointStatus());
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

StemTrackController::~StemTrackController()
{
    //destructor
}
