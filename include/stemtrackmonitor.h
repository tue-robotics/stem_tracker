#ifndef STEMTRACKMONITOR_H
#define STEMTRACKMONITOR_H

#include <map>
#include <string>

class StemRepresentation;
class RobotRepresentation;
class RobotStatus;
class StemTrackController;
class WhiskerGripperInterpreter;
class StemTrackConfigurer;
class RobotInterface;
class VisualizationInterface;

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

    bool m_prev_sample_joint_status_up_to_date;
    bool m_prev_sample_gripper_sensing_up_to_date;
    bool m_debug_state_par, m_find_max_touched_values, m_trial_is_done;
    stemtrack_state_t m_state;

    StemRepresentation* m_p_stem_representation;
    RobotRepresentation* m_p_robot_representation;
    RobotStatus* m_p_robot_status;
    StemTrackController* m_p_stemtrack_control;
    WhiskerGripperInterpreter* m_p_whisker_gripper_interpreter;
    StemTrackConfigurer* m_p_stemtrack_configurer;
    RobotInterface* m_p_robot_interface;
    VisualizationInterface* m_p_visualization_interface;

    bool reachedEndOfStem();
    void updateState();
    bool inputIsUpToDate();
    const std::string stateToString(stemtrack_state_t state) const;
    void reconfigureAndReset();

    void doPreposBehavior();
    void doGraspBehavior();
    void doFollowBehavior();
    void doEndBehaviour();

public:

    StemTrackMonitor(StemRepresentation* p_stem_representation, RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status ,
                     StemTrackController* p_stemtrack_control, WhiskerGripperInterpreter* p_whisker_gripper_interpreter,
                     StemTrackConfigurer* p_stemtrack_configurer, RobotInterface* p_robot_interface, VisualizationInterface* p_visualization_interface)
        : m_p_robot_representation(p_robot_representation), m_p_stem_representation(p_stem_representation), m_p_stemtrack_control(p_stemtrack_control),
          m_p_robot_status(p_robot_status), m_p_whisker_gripper_interpreter(p_whisker_gripper_interpreter), m_p_stemtrack_configurer(p_stemtrack_configurer),
          m_p_robot_interface(p_robot_interface), m_p_visualization_interface(p_visualization_interface), m_state(INIT),
          m_prev_sample_joint_status_up_to_date(true), m_prev_sample_gripper_sensing_up_to_date(true) {}

    inline void setDebugStateParameter(bool debug_state_par) { m_debug_state_par = debug_state_par; }
    inline void setFindMaxTouchedValues(bool find_max_vals) { m_find_max_touched_values = find_max_vals; }
    bool update();

    virtual ~StemTrackMonitor();
};

#endif // STEMTRACKMONITOR_H
