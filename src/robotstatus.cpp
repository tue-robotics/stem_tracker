#include "robotstatus.h"

RobotStatus::RobotStatus(RobotRepresentation* p_robot_representation)
{
    m_n_joints_monitoring = p_robot_representation->getKinematicChain().getNrOfJoints();

    if(m_n_joints_monitoring <= 0)
        INFO_STREAM("trying to initialize robot status object with zero or negative number of joints to monitor!");

    m_joints_to_monitor = KDL::JntArray(m_n_joints_monitoring);
    m_p_robot_representation = p_robot_representation;
}

bool RobotStatus::selfCheck()
{
    bool IamOK = true;

    if(m_n_joints_monitoring <= 0)
    {
        INFO_STREAM("trying to use robot status object while number of joints to monitor is zero or negative!");
        IamOK = false;
    }

    return IamOK;
}

void RobotStatus::updateJointStatus(KDL::JntArray updated_joint_status)
{
    m_joints_to_monitor = updated_joint_status;
    m_last_update = ros::Time::now();
    return;
}

ros::Time RobotStatus::getLastUpdateTime()
{
    return m_last_update;
}

double RobotStatus::getTimeSinceLastUpdate()
{
    // returns n seconds since last update in any of
    // the joints we are monitoring
    ros::Duration interval = ros::Time::now() - RobotStatus::getLastUpdateTime();
    return interval.toSec();
}

bool RobotStatus::isUpToDate()
{
    if(getTimeSinceLastUpdate() < m_up_to_date_threshold)
        return true;
    else
        return false;
}

KDL::JntArray RobotStatus::getJointStatus()
{
    return m_joints_to_monitor;
}

void RobotStatus::setUpToDateThreshold(double threshold)
{
    m_up_to_date_threshold = threshold;
}

std::vector<float> RobotStatus::getGripperXYZ()
{
    KDL::Frame cartpos = getGripperKDLframe();

    m_gripper_xyz.clear();
    m_gripper_xyz.push_back(cartpos.p.x());
    m_gripper_xyz.push_back(cartpos.p.y());
    m_gripper_xyz.push_back(cartpos.p.z());

    return m_gripper_xyz;

}

void RobotStatus::updateGripperXYZ()
{
    KDL::Frame cartpos = getGripperKDLframe();
    m_gripper_xyz.clear();
    m_gripper_xyz.push_back(cartpos.p.x());
    m_gripper_xyz.push_back(cartpos.p.y());
    m_gripper_xyz.push_back(cartpos.p.z());
    return;
}

KDL::Frame RobotStatus::getGripperKDLframe()
{
    KDL::ChainFkSolverPos_recursive forward_kinematics_solver = KDL::ChainFkSolverPos_recursive(m_p_robot_representation->getKinematicChain());

    int fk_ret = forward_kinematics_solver.JntToCart(getJointStatus(),m_gripper_kdlframe);

    if( fk_ret < 0 )
        INFO_STREAM("Warning: something wrong in solving forward kinematics in getGripperKDLframe");

    return m_gripper_kdlframe;
}

bool RobotStatus::isGripperXYZvalid()
{
    if(m_gripper_xyz.size() == 3)
        return true;
    else
        return false;
}

bool RobotStatus::hasValidGripperXYZ()
{
    if( isUpToDate() )
    {
        updateGripperXYZ();
        return isGripperXYZvalid();
    }
    else
        return false;
}

void RobotStatus::printAll()
{
    INFO_STREAM("===============");

    INFO_STREAM("Robot status: ");

    for(int i; i<m_n_joints_monitoring; ++i)
        INFO_STREAM("\t" << m_joints_to_monitor(i));

    INFO_STREAM("===============");
}

RobotStatus::~RobotStatus()
{
    // destructor
}
