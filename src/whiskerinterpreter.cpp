#include "whiskerinterpreter.h"
#include "loggingmacros.h"
#include <cmath>
#include "robotstatus.h"

void WhiskerInterpreter::updateWhiskers(std::vector<float> whisker_values)
{
    for(int i=0; i<m_n_whiskers; ++i)
        m_whisker_values[i] = whisker_values[i];
}

void WhiskerInterpreter::simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center)
{
    m_whisker_forces.clear();
    m_estimated_pos.clear();

    if(gripper_center.size() < 2 )
    {
        ERROR_STREAM("In simulateWhiskerGripper, gripper center xy needed!");
        m_status = 0;
        return;
    }

    if(stem_center.size() < 2 )
    {
        ERROR_STREAM("In simulateWhiskerGripper, stem center xy needed!");
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

    INFO_STREAM("Forces: x = " << tmp_force[0] << " y = " << tmp_force[1] );

    m_whisker_forces.push_back(tmp_force);
}

void WhiskerInterpreter::readWhiskers()
{
    m_whisker_forces.clear();
    m_estimated_pos.clear();

    std::vector<float> tmp_force;
    tmp_force.assign(3,0.0);

    m_estimated_pos.assign(3,0.0);

    std::vector<float> whisker_measurements = m_p_robot_status->getWhiskerMeasurements();

    if( whisker_measurements.size() != m_n_whiskers)
    {
        ERROR_STREAM("In WhiskerInterpreter we expect " << m_n_whiskers << " whiskers while RobotStatus provides " << whisker_measurements.size());
        return;
    }

//    for(int i = 0; i<m_n_whiskers; ++i)
//        INFO_STREAM("Whisker " << i << " has value " << whiskers_state[i]);

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
