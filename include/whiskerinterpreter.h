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
    float m_whisker_length;
    float m_gripper_diameter;
    float m_gripper_radius;
    float m_max_whisker_force;
    int m_status;
    std::vector< std::vector<float> > m_whisker_forces;
    std::vector<float> m_estimated_pos;

    RobotRepresentation* m_p_robot_representation;

public:

    WhiskerInterpreter(RobotRepresentation* p_robot_representation) : m_p_robot_representation(p_robot_representation), m_status(0) {}

    void setGripperDiameter(const float& gripper_diameter);
    void setWhiskerLength(const float& whisker_length);
    void setMaxWhiskerForce(const float& max_whisker_force);
    void setNumberOfWhiskers(const int& n_whiskers);
    void readWhiskers();

    inline const int& getStatus() const { return m_status; }
    inline const std::vector< std::vector<float> >& getWhiskerForces() const { return m_whisker_forces; }
    inline const std::vector<float>& getEstimatedPosError() const { return m_estimated_pos; } //estimated relative position gripper vs stem, assuming gripper is in horizontal plane!
    void simulateWhiskerGripper(const std::vector<float>& gripper_center, const std::vector<float>& stem_center);

    virtual ~WhiskerInterpreter();
};

#endif // WHISKERINTERPRETER_H
