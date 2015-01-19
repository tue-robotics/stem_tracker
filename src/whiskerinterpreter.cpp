#include "whiskerinterpreter.h"
#include <cmath>
#include <ros/ros.h>  /* Only for ROS_ROS_INFO_STREAM */


void WhiskerInterpreter::setNumberOfWhiskers(const int& n_whiskers)
{
    m_n_whiskers = n_whiskers;
}

void WhiskerInterpreter::setGripperDiameter(const float& gripper_diameter)
{
    m_gripper_diameter = gripper_diameter;
    m_gripper_radius = gripper_diameter / 2.0f;
}

void WhiskerInterpreter::setWhiskerLength(const float& whisker_length)
{
    m_whisker_length = whisker_length;
}

void WhiskerInterpreter::setMaxWhiskerForce(const float& max_whisker_force)
{
    m_max_whisker_force = max_whisker_force;
}

bool WhiskerInterpreter::selfCheck() const
{

    bool IamOK = true;

    if (m_n_whiskers <= 0)
    {
        ROS_INFO_STREAM("In whisker gripper with id " << m_gripper_id << " number of whiskers set to zero or negative number");
        IamOK = false;
    }

    if (m_whisker_length > m_gripper_radius)
    {
        ROS_INFO_STREAM("in whisker gripper with id " << m_gripper_id << " length of whiskers is larger than radius of gripper");
        IamOK = false;
    }

    return IamOK;
}

void WhiskerInterpreter::simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center)
{
    m_whisker_force.clear();
    m_estimated_pos_error.clear();

    if(gripper_center.size() < 2 )
    {
        ROS_INFO_STREAM("in simulateWhiskerGripper gripper center xy needed!");
        m_status = 0;
        return;
    }

    if(stem_center.size() < 2 )
    {
        ROS_INFO_STREAM("in simulateWhiskerGripper stem center xy needed!");
        m_status = 0;
        return;
    }

    m_whisker_force.assign(3,0.0);
    m_estimated_pos_error.assign(2,0.0);

    m_estimated_pos_error[0] = gripper_center[0] - stem_center[0];
    m_estimated_pos_error[1] = gripper_center[1] - stem_center[1];

    float dist_gripper_to_stem = sqrt( m_estimated_pos_error[0] * m_estimated_pos_error[0] + m_estimated_pos_error[1] * m_estimated_pos_error[1] );

    if( dist_gripper_to_stem > m_gripper_radius )
    {
        m_status = 1;
    }
    else if (dist_gripper_to_stem < m_gripper_radius - m_whisker_length)
    {
        m_status = 3;
    }
    else
    {
        m_status = 2;

        /* simulated force is inverse of distance to force */
        float whisker_fraction_deformed = ( dist_gripper_to_stem - (m_gripper_radius - m_whisker_length) ) / m_whisker_length;
        m_whisker_force.at(0) = (m_estimated_pos_error[0] / dist_gripper_to_stem) * m_max_whisker_force * whisker_fraction_deformed;
        m_whisker_force.at(1) = (m_estimated_pos_error[1] / dist_gripper_to_stem) * m_max_whisker_force * whisker_fraction_deformed;
    }
    ROS_INFO_STREAM("forces: x = " << m_whisker_force[0] << " y = " << m_whisker_force[1] );

}

void WhiskerInterpreter::readWhiskers()
{
    m_whisker_force.clear();
    m_estimated_pos_error.clear();

    m_whisker_force.assign(3,0.0);
    m_estimated_pos_error.assign(2,0.0);

    std::vector<float> whisker_state = m_p_robot_representation->getWhiskerState();

//    for(int i = 0; i<8; ++i)
//    {
//        ROS_INFO_STREAM("whisker " << i << " has value " << whisker_state[i]);
//    }

    // going counter clockwise, start righthandside closest to amigo, skipped first one
    if( fabs(2.93) - whisker_state[2] > 0.1 )
    {
        ROS_INFO_STREAM("touch 1");
        m_whisker_force[0] = 0.05;
        m_whisker_force[1] = 0.1;

    }
    else if( fabs(1.91) - whisker_state[1] > 0.1 && fabs(3.24) - whisker_state[3] <= 0.03)
    {
        ROS_INFO_STREAM("touch 2");
        m_whisker_force[0] = 0.0;
        m_whisker_force[1] = 0.13;
    }
    else if( fabs(1.91) - whisker_state[1] > 0.1 && fabs(3.24) - whisker_state[3] > 0.03)
    {
        ROS_INFO_STREAM("touch 3");
        m_whisker_force[0] = -0.05;
        m_whisker_force[1] = 0.1;
    }
    else if( fabs(2.67) - whisker_state[4] > 0.08 && fabs(2.93) - whisker_state[6] >= 0.1)
    {
        ROS_INFO_STREAM("touch 4");
        m_whisker_force[0] = -0.05;
        m_whisker_force[1] = -0.1;
    }
    else if( fabs(2.67) - whisker_state[4] > 0.08 && fabs(2.93) - whisker_state[6] < 0.1)
    {
        ROS_INFO_STREAM("touch 5");
        m_whisker_force[0] = 0.0;
        m_whisker_force[1] = 0.1;

    }
    else if( fabs(2.17) - whisker_state[5] > 0.1 && fabs(1.95) - whisker_state[7] > 0.1)
    {
        ROS_INFO_STREAM("touch 6");
        m_whisker_force[0] = 0.05;
        m_whisker_force[1] = -0.1;
    }
    else if( fabs(2.17) - whisker_state[5] <= 0.1 && fabs(1.95) - whisker_state[7] > 0.1)
    {
        ROS_INFO_STREAM("touch 7");
        m_whisker_force[0] = 0.05;
        m_whisker_force[1] = -0.1;
    }


//    m_estimated_pos_error[0] = gripper_center[0] - stem_center[0];
//    m_estimated_pos_error[1] = gripper_center[1] - stem_center[1];



}

WhiskerInterpreter::~WhiskerInterpreter()
{
    //destructor
}
