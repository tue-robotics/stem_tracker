#include <kdl/chainfksolverpos_recursive.hpp>
#include <sensor_msgs/JointState.h>
#include <sys/time.h>

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
    m_last_joint_update_times.assign(m_n_joints_monitoring, 0);
    m_last_whiskers_update = 0;
    m_wait_for_joint_update.assign(m_n_joints_monitoring,true);
    m_wait_for_whiskers_update = true;
    m_wait_for_pressure_sensors_update = true;
    m_last_pressure_sensors_update = 0;
}

void RobotStatus::updateWhiskerMeasurements(std::vector<float> updated_whisker_measurements)
{
    if(updated_whisker_measurements.size() != m_n_whiskers)
    {
        ERROR_STREAM("In RobotStatus received whisker message of length " << updated_whisker_measurements.size() << " , I was expecting length " << m_n_whiskers);
        return;
    }

    for( uint i = 0; i < m_n_whiskers; ++i)
            m_whisker_measurements[i] = updated_whisker_measurements[i];

    m_wait_for_whiskers_update = false;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    m_last_whiskers_update = tp.tv_sec * 1000000 + tp.tv_usec;
    return;
}

void RobotStatus::updatePressureSensorMeasurements(std::vector<float> updated_pressure_sensor_measurements)
{
    if(updated_pressure_sensor_measurements.size() != m_n_pressure_sensors)
    {
        ERROR_STREAM("In RobotStatus received pressure sensor message of length " << updated_pressure_sensor_measurements.size() << " , I was expecting length " << m_n_pressure_sensors);
        return;
    }

    for( uint i = 0; i < m_n_pressure_sensors; ++i)
            m_pressure_sensor_measurements[i] = updated_pressure_sensor_measurements[i];

    m_wait_for_pressure_sensors_update = false;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    m_last_pressure_sensors_update = tp.tv_sec * 1000000 + tp.tv_usec;
    return;
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

bool RobotStatus::reachedPosition(std::vector<float> reference)
{
    if( reference.size() != 3)
    {
        WARNING_STREAM("Check for reachedPosition with vector of " << reference.size() << " elements. I need xyz.");
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
    struct timeval tp;
    gettimeofday(&tp, NULL);

    m_joints_to_monitor = updated_joint_status;
    for(uint i = 0; i < m_n_joints_monitoring; ++i)
    {
        if(joints_updated.at(i) == 1)
        {
            m_last_joint_update_times.at(i) = tp.tv_sec * 1000000 + tp.tv_usec;
            m_wait_for_joint_update.at(i) = false;
        }
    }
    return;
}

bool RobotStatus::waitingForFirstJointStatusUpdate()
{
    for(uint i = 0; i < m_n_joints_monitoring; ++i)
    {
        if( m_wait_for_joint_update.at(i) == true)
            return true;
    }
    return false;
}

const long int RobotStatus::getWorstCaseTimeSinceLastJointUpdate() const
{
    uint i_max = 0;
    long int tempmax = 0, interval, now;
    struct timeval tp;

    for(uint i = 0; i < m_n_joints_monitoring; ++i)
    {
        gettimeofday(&tp, NULL);
        now = tp.tv_sec * 1000000 + tp.tv_usec;
        interval = now - m_last_joint_update_times.at(i);
        if( interval >= tempmax)
        {
            i_max = i;
            tempmax = interval;
        }
    }

    return now - m_last_joint_update_times.at(i_max);
}

const long int RobotStatus::getTimeSinceLastWhiskersUpdate() const
{
    long int now;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    now = tp.tv_sec * 1000000 + tp.tv_usec;
    return now - m_last_whiskers_update;
}

const long int RobotStatus::getTimeSinceLastPressureSensorsUpdate() const
{
    long int now;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    now = tp.tv_sec * 1000000 + tp.tv_usec;
    return now - m_last_pressure_sensors_update;
}

void RobotStatus::resetUpToDateStatus()
{
    m_wait_for_whiskers_update = true;
    m_wait_for_pressure_sensors_update = true;
    m_wait_for_joint_update.assign(m_n_joints_monitoring,true);
}

const bool RobotStatus::isUpToDate()
{
    if(!jointStatusIsUpToDate())
        return false;
    if(!whiskerMeasurementsAreUpToDate())
        return false;
    if(!pressureSensorMeasurementsAreUpToDate())
        return false;

    return true;
}

const bool RobotStatus::jointStatusIsUpToDate()
{
    if(waitingForFirstJointStatusUpdate())
        return false;

    if(getWorstCaseTimeSinceLastJointUpdate() < (long int) (m_joints_up_to_date_threshold * 1000000))
        return true;
    else
        return false;
}

const bool RobotStatus::whiskerMeasurementsAreUpToDate()
{
    if(m_wait_for_whiskers_update)
        return false;

    if(getTimeSinceLastWhiskersUpdate() < (long int) (m_whiskers_up_to_date_threshold * 1000000))
        return true;
    else
        return false;
}

const bool RobotStatus::pressureSensorMeasurementsAreUpToDate()
{
    if(m_wait_for_pressure_sensors_update)
        return false;

    if(getTimeSinceLastPressureSensorsUpdate() < (long int) (m_pressure_sensors_up_to_date_threshold * 1000000))
        return true;
    else
        return false;
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

const KDL::Rotation RobotStatus::getGripperRotation()
{
    KDL::Frame cartpos = getGripperKDLframe();
    return cartpos.M;
}

const std::vector<float> RobotStatus::gripperFrameVectorToBaseFrameVector(std::vector<float> gripper_vector)
{
    /* takes a vector (xy or xyz) in the frame of the gripper and returns a vector xyz in the base frame */

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
    if( jointStatusIsUpToDate() )
    {
        updateGripperXYZ();
        return isGripperXYZvalid();
    }
    else
        return false;
}

RobotStatus::~RobotStatus()
{
    // destructor
}
