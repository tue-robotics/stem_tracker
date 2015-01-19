#ifndef WHISKERINTERPRETER_H
#define WHISKERINTERPRETER_H

#include <vector>
#include <string>

#include "robotrepresentation.h"

/* Status:
 * 0 - unknown
 * 1 - gripper not around stem
 * 2 - gripper around stem but touching whiskers
 * 3 - gripper around stem and not touching whiskers */

class WhiskerInterpreter
{

private:

    int m_n_whiskers;
    int m_gripper_id;
    float m_whisker_length;
    float m_gripper_diameter;
    float m_gripper_radius;
    float m_max_whisker_force;
    int m_status;
    std::vector<float> m_whisker_force;
    std::vector<float> m_estimated_pos_error;
    RobotRepresentation* m_p_robot_representation;

public:

    WhiskerInterpreter(int gripper_id, RobotRepresentation* p_robot_representation) : m_gripper_id(gripper_id), m_p_robot_representation(p_robot_representation), m_status(0) {}

    bool selfCheck() const;

    void setGripperDiameter(const float& gripper_diameter);
    void setWhiskerLength(const float& whisker_length);
    void setMaxWhiskerForce(const float& max_whisker_force);
    void setNumberOfWhiskers(const int& n_whiskers);
    void readWhiskers();

    inline const int& getGripperID() const { return m_gripper_id; }
    inline const int& getStatus() const { return m_status; }
    inline const std::vector<float>& getWhiskerNetForce() const { return m_whisker_force; }
    inline const std::vector<float>& getXYerror() const { return m_estimated_pos_error; }

    void simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center);

    virtual ~WhiskerInterpreter();
};

#endif // WHISKERINTERPRETER_H
