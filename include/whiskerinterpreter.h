#ifndef WHISKERINTERPRETER_H
#define WHISKERINTERPRETER_H

#include <vector>
#include <string>

/* Status:
 * 0 - unknown
 * 1 - gripper not around stem
 * 2 - gripper around stem but touching whiskers
 * 3 - gripper around stem and not touching whiskers */

class RobotStatus;

class WhiskerInterpreter
{

private:

    int m_n_whiskers;
    float m_whisker_length;
    float m_gripper_diameter;
    float m_gripper_radius;
    float m_max_whisker_force;
    int m_status;
    std::vector< std::vector<float> > m_whisker_forces;
    std::vector<float> m_estimated_pos;
    std::vector<float> m_whisker_values;

    RobotStatus* m_p_robot_status;

public:

    WhiskerInterpreter(RobotStatus* p_robot_status) : m_p_robot_status(p_robot_status), m_status(0) {}

    inline void setGripperDiameter(const float& gripper_diameter) { m_gripper_diameter = gripper_diameter; m_gripper_radius = gripper_diameter / 2.0f; }
    inline void setWhiskerLength(const float& whisker_length) { m_whisker_length = whisker_length; }
    inline void setMaxWhiskerForce(const float& max_whisker_force) { m_max_whisker_force = max_whisker_force; }
    inline void setNumberOfWhiskers(const int& n_whiskers) { m_n_whiskers = n_whiskers; }

    void readWhiskers();

    void updateWhiskers(std::vector<float> whisker_values);

    inline const int& getStatus() const { return m_status; }
    inline const std::vector< std::vector<float> >& getWhiskerForces() const { return m_whisker_forces; }
    inline const std::vector<float>& getEstimatedPosError() const { return m_estimated_pos; } //estimated relative position gripper vs stem, assuming gripper is in horizontal plane!
    inline const int& getNumberOfWhiskers() const { return m_n_whiskers; }

    void simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center);

    virtual ~WhiskerInterpreter();
};

#endif // WHISKERINTERPRETER_H
