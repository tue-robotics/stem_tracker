#ifndef STEMTRACKMONITOR_H
#define STEMTRACKMONITOR_H

#include <map>
#include <string>

class StemRepresentation;
class RobotRepresentation;
class RobotStatus;
class StemTrackController;
class WhiskerGripperInterpreter;

enum stemtrack_state_t{
    INIT,
    CALIBRATE,
    PREPOS,
    GRASP,
    FOLLOW,
    LOST,
    END,
    SIDEBRANCH,
    ERROR
};


class StemTrackMonitor
{
    private:
    StemRepresentation* m_p_stem_representation;
    RobotRepresentation* m_p_robot_representation;
    RobotStatus* m_p_robot_status;
    StemTrackController* m_p_stemtrack_control;
    WhiskerGripperInterpreter* m_p_whisker_gripper_interpreter;
    stemtrack_state_t m_state;
    bool m_debug_state_par, m_find_max_touched_values;
    public:
    StemTrackMonitor(StemRepresentation* p_stem_representation, RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status ,
                     StemTrackController* p_stemtrack_control, WhiskerGripperInterpreter* p_whisker_gripper_interpreter)
        : m_p_robot_representation(p_robot_representation), m_p_stem_representation(p_stem_representation), m_p_stemtrack_control(p_stemtrack_control),
          m_p_robot_status(p_robot_status), m_p_whisker_gripper_interpreter(p_whisker_gripper_interpreter), m_state(INIT)  {}

    bool reachedEndOfStem();
    void updateState();
    const std::string stateToString(stemtrack_state_t state) const;

    inline void resetState() { m_state = INIT; }
    inline const stemtrack_state_t getState() { updateState(); return m_state; }

    inline bool lookingForMaxTouchedValues() { return m_find_max_touched_values; }

    inline void setDebugStateParameter(bool debug_state_par) { m_debug_state_par = debug_state_par; }
    inline void setFindMaxTouchedValues(bool find_max_vals) { m_find_max_touched_values = find_max_vals; }

    virtual ~StemTrackMonitor();
};

#endif // STEMTRACKMONITOR_H
