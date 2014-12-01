#ifndef STEMTRACKMONITOR_H
#define STEMTRACKMONITOR_H

#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "stemtrackcontroller.h"

enum{
    STEMTRACK_PREPOS = 0,
    STEMTRACK_GRASP,
    STEMTRACK_FOLLOW,
    STEMTRACK_LOST,
    STEMTRACK_END,
    STEMTRACK_SIDEBRANCH,
    STEMTRACK_ERROR
};

class StemTrackMonitor
{
private:
    StemRepresentation* m_p_stem_representation;
    RobotRepresentation* m_p_robot_representation;
    RobotStatus* m_p_robot_status;
    StemTrackController* m_p_stemtrack_control;
    int m_state;
    bool m_debug_state_par;
public:
    StemTrackMonitor(StemRepresentation* p_stem_representation, RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status , StemTrackController* p_stemtrack_control);
    bool reachedEndOfStem();
    void updateState();

    int getState();

    void setDebugStateParameter(bool debug_state_par);

    virtual ~StemTrackMonitor();
};

#endif // STEMTRACKMONITOR_H
