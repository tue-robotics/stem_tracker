#include "debugfunctions.h"
#include "whiskergripperinterpreter.h"
#include "loggingmacros.h"
#include <cmath>

#include "robotstatus.h"
#include "stemtrackcontroller.h"

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

void WhiskerGripperInterpreter::resetInitialization()
{
    m_has_nominal_values = false;
    m_took_n_samples_for_initialization = 0;
    m_firsttime_in_init = true;
    m_grasp_whisker_touched = false;
    m_prev_sample_whisker_touched = false;
    m_pressure_sensor_is_touched = false;
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
        INFO_STREAM("=================================");
        INFO_STREAM("Storing whisker measurements for moving average...");
        m_firsttime_in_init = false;
    }

    if( m_took_n_samples_for_initialization == m_n_samples_for_initialization)
    {
        m_has_nominal_values = true;

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

void WhiskerGripperInterpreter::findPressureSensorMaxTouchedValues()
{
    if( m_p_robot_status->getPressureSensorMeasurements().size() != m_n_pressure_sensors)
    {
        ERROR_STREAM("In WhiskerGripperInterpreter we expect " << m_n_pressure_sensors << " pressure sensors while RobotStatus provides "
                     << m_p_robot_status->getPressureSensorMeasurements().size());
        return;
    }

    float err;
    std::stringstream ps_max_vals;
    bool changed = false;

    for(uint i = 0; i < m_n_pressure_sensors; ++i)
    {
        err = fabs(m_p_robot_status->getPressureSensorMeasurements()[i]-m_nominal_pressure_sensor_values[i]);

        if( err > m_pressure_sensor_touched_max[i])
        {
            m_pressure_sensor_touched_max[i] = err;
            changed = true;
        }

        ps_max_vals << "p_" << i << "_max: " << m_pressure_sensor_touched_max[i] << "  ";
    }

    if(changed)
        INFO_STREAM(ps_max_vals.str());

    return;
}


void WhiskerGripperInterpreter::checkForWhiskersTouched()
{
    /* Takes latest whisker measurements from RobotStatus and checkes which whiskers are
       touched. If a whisker is touched, the angle it makes wrt the gripper center in the
       gripper xy plane (x-direction is zero) is stored in m_gripper_inside_touched. This
       function also updates the moving average over stored whisker values and sets
       m_grasp_whisker_touched. */

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

    if(!m_prev_sample_whisker_touched && m_gripper_inside_touched_at.size() > 0)
        m_p_stemtrack_controller->setTouchStartedAtXYZ(m_p_robot_status->getGripperXYZ());

    if(m_gripper_inside_touched_at.size() > 0)
        m_prev_sample_whisker_touched = true;
    else
        m_prev_sample_whisker_touched = false;

    return;
}

void WhiskerGripperInterpreter::updateWhiskersTouchedVectors()
{
    /* Takes list of angles defined in gripper frame (around z-axis, x-direction is zero) and turns this
       in a list of vectors containing the origin of each touch vector (xyz at the whisker root) and a list of vectors
       containing the tip of each touched vector (xyz). Uses config par 'gripper radius' to determine
       origin and config par 'whisker length' to determine tip wrt origin */

    for(uint i = 0; i < getWhiskersTouchedAtAngles().size(); ++i)
    {
        if(getWhiskersTouchedAtAngles()[i]>360.0 || getWhiskersTouchedAtAngles()[i]<0.0)
        {
            WARNING_STREAM("Unknown angle in updateWhiskersTouchedAtVectors!");
            return;
        }
    }
    m_whiskers_touched_tips.clear();
    m_whiskers_touched_origins.clear();

    for(uint i = 0; i < getWhiskersTouchedAtAngles().size(); ++i)
    {
        std::vector<float> tip, origin;
        float angle_rad = getWhiskersTouchedAtAngles()[i]/360.0f*2.0f*3.141592f;

        origin.push_back( cos(angle_rad) * ( getWhiskerLength() + getGripperRadius() ) );
        origin.push_back( sin(angle_rad) * ( getWhiskerLength() + getGripperRadius() ) );
        origin.push_back( 0.0 );

        m_whiskers_touched_origins.push_back(origin);

        tip.assign( 3, 0.0 );
        tip.push_back( -cos(angle_rad) * getWhiskerLength() );
        tip.push_back( -sin(angle_rad) * getWhiskerLength() );

        m_whiskers_touched_tips.push_back(tip);
    }

    return;
}

void WhiskerGripperInterpreter::checkForTopSensorTouched()
{
        m_gripper_top_touched_at.clear();
        std::vector<float> topsensor_measurements = m_p_robot_status->getPressureSensorMeasurements();
        if( topsensor_measurements.size() != m_n_pressure_sensors)
        {
            ERROR_STREAM("In WhiskerGripperInterpreter we expect " << m_n_pressure_sensors << " pressure sensors while RobotStatus provided "
                         << topsensor_measurements.size());
            return;
        }

        m_pressure_sensor_is_touched = false;
        for(uint i = 0; i < m_n_pressure_sensors; ++i)
        {
            if( fabs(topsensor_measurements[i]-m_nominal_pressure_sensor_values[i])/m_pressure_sensor_touched_max[i]
                    > m_pressure_sensor_touched_normalized_threshold  )
                m_pressure_sensor_is_touched = true;
        }
       return;
}
void WhiskerGripperInterpreter::updateWhiskerInterpretation()
{
    checkForWhiskersTouched();
    updateWhiskersTouchedVectors();
    updateWhiskerPosErrVecs();
    updateEstimatedPosError();
}

void WhiskerGripperInterpreter::updateWhiskerPosErrVecs()
{
    /* Takes list of angles defined in gripper frame (around z-axis, x-direction is zero) and turns this
       in a list of vectors xyz pointing from the current gripper center to the point of touch. Uses
       config par 'gripper radius' and config par 'whisker length' to determine point of touch given
       a touch angle, assuming a whiskers is pushed to half its original length when touched, assuming
       touch takes place in z = 0 plane */

    m_whisker_pos_err_vectors.clear();
    std::vector<float> vec;
    float angle_rad;

    for(uint i = 0; i < getWhiskersTouchedAtAngles().size(); ++i)
    {
        angle_rad = getWhiskersTouchedAtAngles()[i]/360.0f*2.0f*3.141592f;
        vec.push_back( cos(angle_rad) * (getGripperRadius()+0.5*getWhiskerLength()) );
        vec.push_back( sin(angle_rad) * (getGripperRadius()+0.5*getWhiskerLength()) );
        vec.push_back( 0.0 );
        m_whisker_pos_err_vectors.push_back(vec);
        vec.clear();
    }
    return;
}

void WhiskerGripperInterpreter::updateEstimatedPosError()
{
    /* averages all position errors in m_whisker_pos_err_vectors (possibly multiple because of multiple
       simultaneous whisker touches) to obtain a single position error, assumed to be in the z = 0 plane of the
       gripper */

    m_estimated_pos.clear();
    m_estimated_pos.assign( 3, 0.0 );

    for(uint i = 0; i < getWhiskersPosErrVectors().size(); ++i)
    {
        m_estimated_pos[0] += getWhiskersPosErrVectors()[i][0] / ((float) getWhiskersPosErrVectors().size() );
        m_estimated_pos[1] += getWhiskersPosErrVectors()[i][1] / ((float) getWhiskersPosErrVectors().size() );
    }

    return;
}

WhiskerGripperInterpreter::~WhiskerGripperInterpreter()
{
    //destructor
}
