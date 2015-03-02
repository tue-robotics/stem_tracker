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

void WhiskerGripperInterpreter::updateAverageWhiskerValues(const std::vector<float>& whisker_val)
{
    if( whisker_val.size() != m_n_whiskers )
    {
        ERROR_STREAM("I was expecting " << m_n_whiskers << " whiskers in updateAverageWHiskerValues. I got " << whisker_val.size());
        return;
    }

    m_whisker_values_for_average.push_back( whisker_val );
    for( uint i = 0; i < m_n_whiskers; ++i)
    {
        m_averaged_whisker_values[i] += whisker_val[i] / (float) m_n_samples_for_average
                - m_whisker_values_for_average[0][i] / (float) m_n_samples_for_average;
    }
    m_whisker_values_for_average.erase(m_whisker_values_for_average.begin());

    if( m_whisker_values_for_average.size() != m_n_samples_for_average)
        ERROR_STREAM("I find " << m_whisker_values_for_average.size() << " whiskers in m_whisker_values_for_average. I expected " << m_n_samples_for_average);

    return;
}

void WhiskerGripperInterpreter::obtainNominalValues()
{
    updateAverageWhiskerValues(m_p_robot_status->getWhiskerMeasurements());

    for( uint i = 0; i < m_n_pressure_sensors; ++i)
        m_nominal_pressure_sensor_values.at(i) += m_p_robot_status->getPressureSensorMeasurements().at(i) / (float) m_n_samples_for_initialization;

    ++m_took_n_samples_for_initialization;

    if(m_firsttime_in_init)
    {
        INFO_STREAM("===============================");
        INFO_STREAM("Initializing...");
        m_firsttime_in_init = false;
    }

    if( m_took_n_samples_for_initialization == m_n_samples_for_initialization)
    {
        m_has_nominal_values = true;

        INFO_STREAM("===============================");
        INFO_STREAM("Moving average whisker values: ");
        std::stringstream tmp;
        for( uint i = 0; i < m_n_whiskers; ++i)
            tmp << "w" << i << "= " << m_averaged_whisker_values.at(i) << "  ";
        INFO_STREAM(tmp.str());

        INFO_STREAM("Nominal pressure sensor values: ");
        std::stringstream tmp2;
        for( uint i = 0; i < m_n_pressure_sensors; ++i)
            tmp2 << "p" << i << "= " << m_nominal_pressure_sensor_values.at(i) << "  ";
        INFO_STREAM(tmp2.str());
    }

    return;
}

void WhiskerGripperInterpreter::findWhiskerMaxTouchedValues()
{
    if( m_p_robot_status->getWhiskerMeasurements().size() != m_n_whiskers)
    {
        ERROR_STREAM("In WhiskerGripperInterpreter we expect " << m_n_whiskers << " whiskers while RobotStatus provides "
                     << m_p_robot_status->getWhiskerMeasurements().size());
        return;
    }

    float err;
    std::stringstream w_max_vals;
    bool changed = false;

    for(uint i = 0; i < m_n_whiskers; ++i)
    {
        err = fabs(m_p_robot_status->getWhiskerMeasurements()[i]-m_averaged_whisker_values[i]);
        if( err > m_whisker_touched_max[i])
        {
            m_whisker_touched_max[i] = err;
            changed = true;
        }

        w_max_vals << "w_" << i << "_max: " << m_whisker_touched_max[i] << "  ";
    }

    if(changed)
        INFO_STREAM(w_max_vals.str());

    return;
}

void WhiskerGripperInterpreter::readWhiskers()
{
    m_gripper_inside_touched_at.clear();
    m_grasp_whisker_touched = false;

    updateAverageWhiskerValues(m_p_robot_status->getWhiskerMeasurements());

    if( m_p_robot_status->getWhiskerMeasurements().size() != m_n_whiskers)
    {
        ERROR_STREAM("In WhiskerGripperInterpreter we expect " << m_n_whiskers << " whiskers while RobotStatus provides "
                     << m_p_robot_status->getWhiskerMeasurements().size());
        return;
    }

    bool touched; uint n_done = 0;
    for(uint i = 0; i < m_n_whisker_units; ++i)
    {
        touched = false;
        for(uint j = 0; j < m_n_whiskers_per_unit[i]; ++j)
        {
            if(m_whisker_touched_max[j+n_done] <= 0.0)
                ERROR_STREAM("For whisker " << j+n_done << " touched_max is negative!");

            if(fabs(m_p_robot_status->getWhiskerMeasurements()[j+n_done]-m_averaged_whisker_values[j+n_done]) / m_whisker_touched_max[j+n_done]
                                                    > m_normalized_whisker_touched_threshold)
            {
                touched = true;
                if(m_whisker_unit_is_grasp_check[i])
                    m_grasp_whisker_touched = true;
            }
        }
        n_done += m_n_whiskers_per_unit[i];

        if(touched)
        {
            if(m_min_whisker_unit_area_covered[i] > m_max_whisker_unit_area_covered[i])
                m_gripper_inside_touched_at.push_back( fmod((m_min_whisker_unit_area_covered[i]+m_max_whisker_unit_area_covered[i]+360.0)/2.0f,360.0) );
            else
                m_gripper_inside_touched_at.push_back( (m_min_whisker_unit_area_covered[i]+m_max_whisker_unit_area_covered[i])/2.0f );
        }

    }

    return;
}

void WhiskerGripperInterpreter::readTopSensor()
{
    //    m_gripper_top_touched_at.clear();
    //    std::vector<float> topsensor_measurements = m_p_robot_status->getPressureSensorMeasurements();
    //    if( topsensor_measurements.size() != m_n_pressure_sensors)
    //    {
    //        ERROR_STREAM("In WhiskerGripperInterpreter we expect " << m_n_pressure_sensors << " pressure sensors while RobotStatus provided " << topsensor_measurements.size());
    //        return;
    //    }

    //    float frac;
    //    float max_pressuresensor_reach = 1.0;

    //    /* left cover */
    //    if( fabs(topsensor_measurements[0]-m_nominal_pressure_sensor_values[0])>m_pressure_sensor_touched_thresholds ||
    //            fabs(topsensor_measurements[1]-m_nominal_pressure_sensor_values[1])>m_pressure_sensor_touched_thresholds )
    //    {
    //        frac = (fabs(topsensor_measurements[0]-m_nominal_pressure_sensor_values[0])-fabs(topsensor_measurements[1]-
    //                                                                                         m_nominal_pressure_sensor_values[1]))/max_pressuresensor_reach;

    //    }

    //    /* middle cover */
    //    if( fabs(topsensor_measurements[2]-m_nominal_pressure_sensor_values[2])>m_pressure_sensor_touched_thresholds)
    //    {
    //        m_gripper_top_touched_at.push_back( (m_pressure_sensor_covers_min + m_pressure_sensor_covers_max)/2.0f );
    //    }

    //    /* rear cover */
    //    if( fabs(topsensor_measurements[3]-m_nominal_pressure_sensor_values[3])>m_pressure_sensor_touched_thresholds ||
    //            fabs(topsensor_measurements[4]-m_nominal_pressure_sensor_values[4])>m_pressure_sensor_touched_thresholds )
    //    {

    //    }

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

    INFO_STREAM("in updateEstimatedPosErr");
    for(uint i = 0; i < touch_vec.size(); ++i)
    {
        printVector(touch_vec[i]);
        printVector(m_estimated_pos);

        m_estimated_pos[0] += touch_vec[i][0] / ((float) touch_vec.size() );
        m_estimated_pos[1] += touch_vec[i][1] / ((float) touch_vec.size() );

        printVector(m_estimated_pos);
    }

    return;
}

WhiskerGripperInterpreter::~WhiskerGripperInterpreter()
{
    //destructor
}
