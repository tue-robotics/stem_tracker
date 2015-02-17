#ifndef WHISKERINTERPRETER_H
#define WHISKERINTERPRETER_H

#include <vector>
#include <string>

class RobotStatus;

class WhiskerInterpreter
{

private:

    int m_n_whiskers, m_n_pressure_sensors;
    float m_whisker_length;
    float m_gripper_diameter;
    float m_gripper_radius;
    float m_max_whisker_force;
    bool m_has_nominal_values;
    std::vector<float> m_nominal_whisker_values, m_nominal_pressure_sensor_values;
    std::vector< std::vector<float> > m_whisker_forces;
    std::vector<float> m_estimated_pos;
    std::vector<float> m_whisker_values;
    int m_took_n_samples_for_initialization;
    int m_n_samples_for_initialization;

    RobotStatus* m_p_robot_status;

public:

    WhiskerInterpreter(RobotStatus* p_robot_status) : m_p_robot_status(p_robot_status), m_has_nominal_values(false), m_took_n_samples_for_initialization(0) {}

    inline void setGripperDiameter(const float& gripper_diameter) { m_gripper_diameter = gripper_diameter; m_gripper_radius = gripper_diameter / 2.0f; }
    inline void setWhiskerLength(const float& whisker_length) { m_whisker_length = whisker_length; }
    inline void setMaxWhiskerForce(const float& max_whisker_force) { m_max_whisker_force = max_whisker_force; }
    inline void setNumberOfWhiskers(const int& n) { m_n_whiskers = n; m_nominal_whisker_values.assign(n,0.0); }
    inline void setNumberOfPressureSensors(const int& n) { m_n_pressure_sensors = n; m_nominal_pressure_sensor_values.assign(n,0.0); }
    inline void setNumberOfSamplesForInitialization(const int& n_samples) { m_n_samples_for_initialization = n_samples; }

    void readWhiskers();
    void obtainNominalValues();
    void updateWhiskers(std::vector<float> whisker_values);

    inline const bool isInitialized() const { return m_has_nominal_values; }
    inline const std::vector< std::vector<float> >& getWhiskerForces() const { return m_whisker_forces; }
    inline const std::vector<float>& getEstimatedPosError() const { return m_estimated_pos; } //estimated relative position gripper vs stem, assuming gripper is in horizontal plane!
    inline const int& getNumberOfWhiskers() const { return m_n_whiskers; }

    void simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center);

    virtual ~WhiskerInterpreter();
};

#endif // WHISKERINTERPRETER_H
