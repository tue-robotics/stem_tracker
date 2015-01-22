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

void WhiskerInterpreter::simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center)
{
    m_whisker_forces.clear();
    m_estimated_pos.clear();

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

    std::vector<float> tmp_force;
    tmp_force.assign(3,0.0);

    m_estimated_pos.assign(3,0.0);

    m_estimated_pos[0] = gripper_center[0] - stem_center[0];
    m_estimated_pos[1] = gripper_center[1] - stem_center[1];

    float dist_gripper_to_stem = sqrt( m_estimated_pos[0] * m_estimated_pos[0] + m_estimated_pos[1] * m_estimated_pos[1] );

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

        /* simulated force is inverse of distance to stem */
        float whisker_fraction_deformed = ( dist_gripper_to_stem - (m_gripper_radius - m_whisker_length) ) / m_whisker_length;
        tmp_force.at(0) = (m_estimated_pos[0] / dist_gripper_to_stem) * m_max_whisker_force * whisker_fraction_deformed;
        tmp_force.at(1) = (m_estimated_pos[1] / dist_gripper_to_stem) * m_max_whisker_force * whisker_fraction_deformed;
    }
    ROS_INFO_STREAM("forces: x = " << tmp_force[0] << " y = " << tmp_force[1] );
    m_whisker_forces.push_back(tmp_force);
}

void WhiskerInterpreter::readWhiskers()
{
    m_whisker_forces.clear();
    m_estimated_pos.clear();

    std::vector<float> tmp_force;
    tmp_force.assign(3,0.0);

    m_estimated_pos.assign(3,0.0);

    std::vector<float> whiskers_state = m_p_robot_representation->getWhiskersState();

    if( whiskers_state.size() != m_n_whiskers)
    {
        ROS_ERROR_STREAM("readWhiskers in WhiskerInterpreter expects " << m_n_whiskers << " whiskers while RobotInterface provides " << whiskers_state.size() );
        return;
    }

    for(int i = 0; i<m_n_whiskers; ++i)
        ROS_INFO_STREAM("whisker " << i << " has value " << whiskers_state[i]);

//    if( fabs(2.93) - whiskers_state[2] > 0.1 )
//    {
//        ROS_INFO_STREAM("touch 1");
//        m_whisker_force[0] = 0.05;
//        m_whisker_force[1] = 0.1;

//    }
//    else if( fabs(1.91) - whisker_state[1] > 0.1 && fabs(3.24) - whisker_state[3] <= 0.03)
//    {
//        ROS_INFO_STREAM("touch 2");
//        m_whisker_force[0] = 0.0;
//        m_whisker_force[1] = 0.13;
//    }
//    else if( fabs(1.91) - whisker_state[1] > 0.1 && fabs(3.24) - whisker_state[3] > 0.03)
//    {
//        ROS_INFO_STREAM("touch 3");
//        m_whisker_force[0] = -0.05;
//        m_whisker_force[1] = 0.1;
//    }
//    else if( fabs(2.67) - whisker_state[4] > 0.08 && fabs(2.93) - whisker_state[6] >= 0.1)
//    {
//        ROS_INFO_STREAM("touch 4");
//        m_whisker_force[0] = -0.05;
//        m_whisker_force[1] = -0.1;
//    }
//    else if( fabs(2.67) - whisker_state[4] > 0.08 && fabs(2.93) - whisker_state[6] < 0.1)
//    {
//        ROS_INFO_STREAM("touch 5");
//        m_whisker_force[0] = 0.0;
//        m_whisker_force[1] = 0.1;

//    }
//    else if( fabs(2.17) - whisker_state[5] > 0.1 && fabs(1.95) - whisker_state[7] > 0.1)
//    {
//        ROS_INFO_STREAM("touch 6");
//        m_whisker_force[0] = 0.05;
//        m_whisker_force[1] = -0.1;
//    }
//    else if( fabs(2.17) - whisker_state[5] <= 0.1 && fabs(1.95) - whisker_state[7] > 0.1)
//    {
//        ROS_INFO_STREAM("touch 7");
//        m_whisker_force[0] = 0.05;
//        m_whisker_force[1] = -0.1;
//    }
    for(uint i = 0; i < m_whisker_forces.size(); ++i)
    {
        m_estimated_pos[0] += m_whisker_forces[i][0];
        m_estimated_pos[1] += m_whisker_forces[i][1];
    }

    if( m_whisker_forces.size() > 0 )
    {
        m_estimated_pos[0] /= m_whisker_forces.size();
        m_estimated_pos[1] /= m_whisker_forces.size();
    }
}

WhiskerInterpreter::~WhiskerInterpreter()
{
    //destructor
}
