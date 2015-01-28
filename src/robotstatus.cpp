#include <kdl/chainfksolverpos_recursive.hpp>
#include <sensor_msgs/JointState.h>

#include "robotstatus.h"
#include "loggingmacros.h"
#include "robotrepresentation.h"

RobotStatus::RobotStatus(RobotRepresentation* p_robot_representation)
{
    m_n_joints_monitoring = p_robot_representation->getKinematicChain().getNrOfJoints();

    if(m_n_joints_monitoring <= 0)
        ERROR_STREAM("Trying to initialize RobotStatus object with zero or a negative number of joints to monitor!");

    m_joints_to_monitor = KDL::JntArray(m_n_joints_monitoring);
    m_p_robot_representation = p_robot_representation;
    m_pos_reached_threshold = -1.0;
    m_xyz_reached_threshold = -1.0;
    m_last_update_times.resize(m_n_joints_monitoring);
    m_starting_up.assign(m_n_joints_monitoring,true);
}

void RobotStatus::setPosReachedThreshold(float pos_reached_threshold)
{
    m_pos_reached_threshold = pos_reached_threshold;
}

bool RobotStatus::reachedPosition(KDL::JntArray reference)
{
    if( reference.rows() != m_n_joints_monitoring )
    {
        ERROR_STREAM("reachedPosition check for " << reference.rows() << " joints while robot status contains " << m_n_joints_monitoring << " joints");
        return false;
    }

    if( m_pos_reached_threshold < 0.0)
    {
        ERROR_STREAM("Check for reachedPosition check while threshold was not set!");
        return false;
    }

    for( int i = 0; i < m_n_joints_monitoring; ++i)
    {
        if( fabs( reference(i) - m_joints_to_monitor(i)) > m_pos_reached_threshold )
            return false;
    }

    return true;
}

double RobotStatus::setXYZreachedThreshold(double xyz_reached_threshold)
{
    m_xyz_reached_threshold = xyz_reached_threshold;
}

bool RobotStatus::reachedPosition(std::vector<float> reference)
{
    if( reference.size() != 3)
    {
        ERROR_STREAM("Check for reachedPosition with vector of " << reference.size() << " elements. I need xyz.");
        return false;
    }

    if( m_xyz_reached_threshold < 0.0)
    {
        ERROR_STREAM("Check for reachedPosition while threshold was not set!");
        return false;
    }

    if( getGripperXYZ().size() != 3)
    {
        ERROR_STREAM("Check for reachedPosition while gripperxyz is not known!");
        return false;
    }

    double diff[3] = {getGripperXYZ()[0] - reference[0], getGripperXYZ()[1] - reference[1], getGripperXYZ()[2] - reference[2]};

    if( diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2] > m_xyz_reached_threshold * m_xyz_reached_threshold)
        return false;
    else
        return true;

}

void RobotStatus::updateJointStatus(KDL::JntArray updated_joint_status, std::vector<int> joints_updated)
{
    m_joints_to_monitor = updated_joint_status;
    for(uint i = 0; i < m_n_joints_monitoring; ++i)
    {
        if(joints_updated.at(i) == 1)
        {
            m_last_update_times.at(i) = ros::Time::now();
            m_starting_up.at(i) = false;
        }
    }
    return;
}

bool RobotStatus::waitingForFirstStatusUpdate()
{
    for(uint i = 0; i < m_n_joints_monitoring; ++i)
    {
        if( m_starting_up.at(i) == true)
            return true;
    }
    return false;
}

double RobotStatus::getWorstCaseTimeSinceLastUpdate()
{
    uint i_max = 0;
    double tempmax = 0.0;
    ros::Duration interval;
    for(uint i = 0; i < m_n_joints_monitoring; ++i)
    {
        interval = ros::Time::now() - m_last_update_times.at(i);
        if(interval.toSec() >= tempmax)
        {
            i_max = i;
            tempmax = interval.toSec();
        }
    }
    interval = ros::Time::now() - m_last_update_times.at(i_max);
    return interval.toSec();
}

bool RobotStatus::isUpToDate()
{
    if(waitingForFirstStatusUpdate())
        return false;

    if(getWorstCaseTimeSinceLastUpdate() < m_up_to_date_threshold)
        return true;
    else
        return false;
}

void RobotStatus::setUpToDateThreshold(double threshold)
{
    m_up_to_date_threshold = threshold;
}

const std::vector<float>& RobotStatus::getGripperXYZ()
{
    updateGripperXYZ();
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

const KDL::Frame& RobotStatus::getGripperKDLframe()
{
    KDL::ChainFkSolverPos_recursive forward_kinematics_solver = KDL::ChainFkSolverPos_recursive(m_p_robot_representation->getKinematicChain());

    int fk_ret = forward_kinematics_solver.JntToCart(getJointStatus(),m_gripper_kdlframe);

    if( fk_ret < 0 )
        WARNING_STREAM("Something wrong in solving forward kinematics in getGripperKDLframe.");

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
