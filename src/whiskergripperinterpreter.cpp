#include "debugfunctions.h"
#include "whiskergripperinterpreter.h"
#include "loggingmacros.h"
#include <cmath>
#include "robotstatus.h"

void WhiskerGripperInterpreter::simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center)
{
    m_estimated_pos.clear();

    if(gripper_center.size() < 2 )
    {
        ERROR_STREAM("In simulateWhiskerGripper, gripper center xy needed!");
        return;
    }

    if(stem_center.size() < 2 )
    {
        ERROR_STREAM("In simulateWhiskerGripper, stem center xy needed!");
        return;
    }

    m_estimated_pos.assign(3,0.0);
    m_estimated_pos[0] = gripper_center[0] - stem_center[0];
    m_estimated_pos[1] = gripper_center[1] - stem_center[1];

    float dist_gripper_to_stem = sqrt( m_estimated_pos[0] * m_estimated_pos[0] + m_estimated_pos[1] * m_estimated_pos[1] );

    if( dist_gripper_to_stem > m_gripper_radius || dist_gripper_to_stem < m_gripper_radius - m_whisker_length)
    {
        /* pos err not large enough to be noticed by whiskers, or stem not in gripper */
        m_estimated_pos.assign(3,0.0);
    }

    return;
}

void WhiskerGripperInterpreter::obtainNominalValues()
{
    for( uint i = 0; i < m_n_whiskers; ++i)
        m_nominal_whisker_values.at(i) += m_p_robot_status->getWhiskerMeasurements().at(i) / (float) m_n_samples_for_initialization;

    for( uint i = 0; i < m_n_pressure_sensors; ++i)
        m_nominal_pressure_sensor_values.at(i) += m_p_robot_status->getPressureSensorMeasurements().at(i) / (float) m_n_samples_for_initialization;

    ++m_took_n_samples_for_initialization;

    if( m_took_n_samples_for_initialization == m_n_samples_for_initialization)
    {
        m_has_nominal_values = true;

        INFO_STREAM("===============================");
        INFO_STREAM("Nominal whisker values: ");
        std::stringstream tmp;
        for( uint i = 0; i < m_n_whiskers; ++i)
            tmp << "w" << i << "= " << m_nominal_whisker_values.at(i) << "  ";
        INFO_STREAM(tmp.str());

        INFO_STREAM("Nominal pressure sensor values: ");
        std::stringstream tmp2;
        for( uint i = 0; i < m_n_pressure_sensors; ++i)
            tmp2 << "p" << i << "= " << m_nominal_pressure_sensor_values.at(i) << "  ";
        INFO_STREAM(tmp2.str());
    }

    return;
}

void WhiskerGripperInterpreter::readWhiskers()
{
    m_gripper_inside_touched_at.clear();

    std::vector<float> whisker_measurements = m_p_robot_status->getWhiskerMeasurements();

    if( whisker_measurements.size() != m_n_whiskers)
    {
        ERROR_STREAM("In WhiskerGripperInterpreter we expect " << m_n_whiskers << " whiskers while RobotStatus provides " << whisker_measurements.size());
        return;
    }

    for(uint i = 0; i < m_n_whiskers; ++i)
    {
        if(fabs(whisker_measurements[i]-m_nominal_whisker_values[i]) > m_whisker_touched_threshold)
        {
            if(m_min_whisker_area_covered[i] > m_max_whisker_area_covered[i])
                m_gripper_inside_touched_at.push_back( fmod((m_min_whisker_area_covered[i]+m_max_whisker_area_covered[i]+360.0)/2.0f,360.0) );
            else
                m_gripper_inside_touched_at.push_back( (m_min_whisker_area_covered[i]+m_max_whisker_area_covered[i])/2.0f );
        }
    }

    return;
}

std::vector< std::vector<float> > WhiskerGripperInterpreter::touchAngleToVect(const std::vector<float>& angles, const float& length) const
{
    std::vector<float> vec;
    std::vector< std::vector<float> > vecs;

    for(uint i = 0; i < angles.size(); ++i)
    {
        float angle_rad = angles[i]/360.0f*2.0f*3.141592f;
        vec.push_back(- cos(angle_rad)*length);
        vec.push_back(- sin(angle_rad)*length);
        vecs.push_back(vec);
        vec.clear();
    }
    return vecs;
}

void WhiskerGripperInterpreter::updateEstimatedPosError()
{
    m_estimated_pos.clear();
    m_estimated_pos.assign(3,0.0);

    std::vector< std::vector<float> > touch_vec = touchAngleToVect( getWhiskersTouchedAt(), 1.0);

    for(uint i = 0; i < touch_vec.size(); ++i)
    {
        m_estimated_pos[0] += touch_vec[i][0] / ((float) touch_vec.size() );
        m_estimated_pos[1] += touch_vec[i][1] / ((float) touch_vec.size() );
    }
}

bool WhiskerGripperInterpreter::graspWhiskerIsTouched()
{
    std::vector<float> whisker_measurements = m_p_robot_status->getWhiskerMeasurements();

    if( whisker_measurements.size() != m_n_whiskers)
    {
        ERROR_STREAM("In WhiskerGripperInterpreter we expect " << m_n_whiskers << " whiskers while RobotStatus provides " << whisker_measurements.size());
        return false;
    }

    for(uint i = 0; i < m_n_whiskers; ++i)
    {
        if(fabs(whisker_measurements[i]-m_nominal_whisker_values[i]) > m_whisker_touched_threshold && m_whisker_is_grasp_check[i] == true)
            return true;
    }

    return false;
}

WhiskerGripperInterpreter::~WhiskerGripperInterpreter()
{
    //destructor
}
