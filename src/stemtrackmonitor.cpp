#include "stemtrackmonitor.h"

StemTrackMonitor::StemTrackMonitor(StemRepresentation* p_stem_representation, RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status)
{
    m_p_robot_representation = p_robot_representation;
    m_p_stem_representation = p_stem_representation;
    m_p_robot_status = p_robot_status;
    m_state = STEMTRACK_PREPOS;
}

void StemTrackMonitor::setDebugStateParameter(bool debug_state_par)
{
    m_debug_state_par = debug_state_par;
}

bool StemTrackMonitor::reachedEndOfStem(int up)
{
    if( m_p_stem_representation->getNearestXYZ().size() != 3)
    {
        INFO_STREAM("trying to check for end of stem while stem-intersection is not known");
    }

    if( fabs(m_p_stem_representation->getNearestXYZ().at(2) - m_p_stem_representation->getNodesZ().back()) < 0.05 && up > 0 )
        return true;
    else if ( fabs(m_p_stem_representation->getNearestXYZ().at(2) - m_p_stem_representation->getNodesZ().front() ) < 0.05 && up < 0 )
        return true;
    else
        return false;
}

void StemTrackMonitor::updateState()
{
    int state_prev = m_state;

    if(m_state == STEMTRACK_PREPOS)
    {
        if( m_p_robot_status->reachedPosition( m_p_robot_representation->getInitialPoseJointRefs() ) )
        {
            m_state = STEMTRACK_GRASP;
            INFO_STREAM("===========================");
            INFO_STREAM("Ready for stem tracking");
            INFO_STREAM("===========================");
        }
    }

    if(m_debug_state_par && m_state != state_prev)
        INFO_STREAM("in StemTrackMonitor.updateState(), state was: " << state_prev << " now set to: " << m_state);

}

int StemTrackMonitor::getState()
{
    return m_state;
}

StemTrackMonitor::~StemTrackMonitor()
{
    //destructor
}
