#ifndef WHISKERGRIPPERINTERPRETER_H
#define WHISKERGRIPPERINTERPRETER_H

#include <vector>
#include <string>

class RobotStatus;

class WhiskerGripperInterpreter
{

private:

    int m_n_whiskers, m_n_pressure_sensors, m_n_whisker_units, m_n_samples_for_average;
    std::vector< std::vector<float> > m_whisker_values_for_average;
    float m_whisker_length, m_gripper_radius, m_normalized_whisker_touched_threshold;
    bool m_has_nominal_values, m_grasp_whisker_touched;
    std::vector<float> m_nominal_pressure_sensor_values, m_averaged_whisker_values;
    std::vector<float> m_min_whisker_unit_area_covered, m_max_whisker_unit_area_covered;
    std::vector<float> m_gripper_inside_touched_at, m_gripper_top_touched_at;
    std::vector<bool> m_whisker_unit_is_grasp_check;
    std::vector<float> m_estimated_pos;
    int m_took_n_samples_for_initialization;
    int m_n_samples_for_initialization;
    std::vector<float> m_pressure_sensor_covers_min, m_pressure_sensor_covers_max, m_pressure_sensors_at;
    float m_pressure_sensor_touched_threshold;
    std::vector<float> m_whisker_touched_max;
    std::vector<int> m_n_whiskers_per_unit;
    bool m_firsttime_in_init;

    RobotStatus* m_p_robot_status;

public:

    WhiskerGripperInterpreter(RobotStatus* p_robot_status) : m_p_robot_status(p_robot_status), m_has_nominal_values(false),
        m_took_n_samples_for_initialization(0), m_firsttime_in_init(true), m_grasp_whisker_touched(false) {}

    inline void setGripperDiameter(const float& gripper_diameter) { m_gripper_radius = gripper_diameter / 2.0f; }
    inline void setWhiskerLength(const float& whisker_length) { m_whisker_length = whisker_length; }
    inline void setNumberOfWhiskers(const int& n) { m_n_whiskers = n; m_averaged_whisker_values.assign(n,0.0); m_whisker_touched_max.assign(n,0.0); }
    inline void setNumberOfWhiskerUnits(const int& n) { m_n_whisker_units = n; }
    inline void setNumberOfPressureSensors(const int& n) { m_n_pressure_sensors = n; m_nominal_pressure_sensor_values.assign(n,0.0); }
    inline void setNumberOfSamplesForInitialization(const int& n_samples) { m_n_samples_for_initialization = n_samples; }
    inline void setWhiskerUnitIsGraspCheck(const std::vector<bool> grasp_check) { m_whisker_unit_is_grasp_check = grasp_check; }
    inline void setWhiskerUnitCoversAreaMin(const std::vector<float> min) { m_min_whisker_unit_area_covered = min; }
    inline void setWhiskerUnitCoversAreaMax(const std::vector<float> max) { m_max_whisker_unit_area_covered = max; }
    inline void setWhiskerTouchedMax(const std::vector<float> max) { m_whisker_touched_max = max; }
    inline void setPressureSensorTouchedThreshold(const float threshold) { m_pressure_sensor_touched_threshold = threshold; }
    inline void setPressureSensorsAt(const std::vector<float> loc) { m_pressure_sensors_at = loc; }
    inline void setPressureSensorCoversMin(const std::vector<float> min) { m_pressure_sensor_covers_min = min; }
    inline void setPressureSensorCoversMax(const std::vector<float> max) { m_pressure_sensor_covers_max = max; }
    inline void setNormalizedWhiskerTouchThreshold(const float threshold) { m_normalized_whisker_touched_threshold = threshold; }
    inline void setNumberOfWhiskersPerUnit(const std::vector<int> n_per_unit) { m_n_whiskers_per_unit = n_per_unit; }
    inline void setNumberOfSamplesForMovingAverage(const int n_samples, const int n_whiskers) { m_n_samples_for_average = n_samples;
                                                           m_whisker_values_for_average.assign(n_samples,std::vector<float>(n_whiskers,0.0)); }

    void readWhiskers();
    void readTopSensor();
    void obtainNominalValues();
    void updateEstimatedPosError();
    void updateAverageWhiskerValues(const std::vector<float>& whisker_val);
    void findWhiskerMaxTouchedValues();
    std::vector< std::vector<float> > touchAngleToVect(const std::vector<float>& angles, const float& length) const;

    inline bool graspWhiskerIsTouched() { return m_grasp_whisker_touched; }
    inline const bool isInitialized() const { return m_has_nominal_values; }
    inline const std::vector<float>& getWhiskersTouchedAt() const { return m_gripper_inside_touched_at; }
    inline const std::vector<float>& getPressureSensorsTouchedAt() const { return m_gripper_top_touched_at; }
    inline const std::vector<float>& getEstimatedPosError() const { return m_estimated_pos; }
    inline const int& getNumberOfWhiskers() const { return m_n_whiskers; }

    void simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center);

    virtual ~WhiskerGripperInterpreter();
};

#endif // WHISKERGRIPPERINTERPRETER_H
