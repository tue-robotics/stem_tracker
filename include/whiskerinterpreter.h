#ifndef WHISKERINTERPRETER_H
#define WHISKERINTERPRETER_H

#include <vector>
#include <string>

class RobotStatus;

class WhiskerInterpreter
{

private:

    int m_n_whiskers, m_n_pressure_sensors;
    float m_whisker_length, m_gripper_radius;
    bool m_has_nominal_values;
    std::vector<float> m_nominal_whisker_values, m_nominal_pressure_sensor_values;
    std::vector<float> m_min_whisker_area_covered, m_max_whisker_area_covered;
    std::vector<float> m_gripper_inside_touched_at, m_gripper_top_touched_at;
    std::vector<bool> m_whisker_is_grasp_check;
    std::vector<float> m_estimated_pos;
    int m_took_n_samples_for_initialization;
    int m_n_samples_for_initialization;
    float m_whisker_touched_threshold, m_pressure_sensor_touched_threshold;

    RobotStatus* m_p_robot_status;

public:

    WhiskerInterpreter(RobotStatus* p_robot_status) : m_p_robot_status(p_robot_status), m_has_nominal_values(false), m_took_n_samples_for_initialization(0) {}

    inline void setGripperDiameter(const float& gripper_diameter) { m_gripper_radius = gripper_diameter / 2.0f; }
    inline void setWhiskerLength(const float& whisker_length) { m_whisker_length = whisker_length; }
    inline void setNumberOfWhiskers(const int& n) { m_n_whiskers = n; m_nominal_whisker_values.assign(n,0.0); }
    inline void setNumberOfPressureSensors(const int& n) { m_n_pressure_sensors = n; m_nominal_pressure_sensor_values.assign(n,0.0); }
    inline void setNumberOfSamplesForInitialization(const int& n_samples) { m_n_samples_for_initialization = n_samples; }
    inline void setWhiskerIsGraspCheck(const std::vector<bool> grasp_check) { m_whisker_is_grasp_check = grasp_check; }
    inline void setWhiskerCoversAreaMin(const std::vector<float> min) { m_min_whisker_area_covered = min; }
    inline void setWhiskerCoversAreaMax(const std::vector<float> max) { m_max_whisker_area_covered = max; }
    inline void setWhiskerTouchedThreshold(const float threshold) { m_whisker_touched_threshold = threshold; }
    inline void setPressureSensorTouchedThreshold(const float threshold) { m_pressure_sensor_touched_threshold = threshold; }

    void readWhiskers();
    void obtainNominalValues();

    inline const bool isInitialized() const { return m_has_nominal_values; }
    inline const std::vector<float>& getWhiskersTouchedAt() const { return m_gripper_inside_touched_at; }
    inline const std::vector<float>& getPressureSensorsTouchedAt() const { return m_gripper_top_touched_at; }
    inline const std::vector<float>& getEstimatedPosError() const { return m_estimated_pos; }
    inline const int& getNumberOfWhiskers() const { return m_n_whiskers; }

    void simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center);

    virtual ~WhiskerInterpreter();
};

#endif // WHISKERINTERPRETER_H
