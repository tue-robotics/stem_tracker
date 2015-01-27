#include "stemtrackmonitor.h"
#include "loggingmacros.h"

StemTrackMonitor::StemTrackMonitor(StemRepresentation* p_stem_representation, RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status, StemTrackController* p_stemtrack_control)
{
    m_p_robot_representation = p_robot_representation;
    m_p_stem_representation = p_stem_representation;
    m_p_stemtrack_control = p_stemtrack_control;
    m_p_robot_status = p_robot_status;
    m_state = STEMTRACK_STATE_PREPOS;
}

void StemTrackMonitor::setDebugStateParameter(bool debug_state_par)
{
    m_debug_state_par = debug_state_par;
}

bool StemTrackMonitor::reachedEndOfStem()
{
    if( m_p_stem_representation->getNearestXYZ().size() != 3)
    {
        ERROR_STREAM("Trying to check for end of stem while stem-intersection is not known!");
    }

    if( fabs(m_p_stem_representation->getNearestXYZ().at(2) - m_p_stem_representation->getNodesZ().back()) < 0.05 )
        return true;
    else
        return false;
}

void StemTrackMonitor::updateState()
{
    int state_prev = m_state;

    switch (m_state)
    {

    case STEMTRACK_STATE_PREPOS:
        if( m_p_robot_status->reachedPosition( m_p_robot_representation->getInitialPoseJointRefs() ) )
        {
            m_state = STEMTRACK_STATE_GRASP;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> I am going to grasp the stem, hihaa");
        }
        break;

    case STEMTRACK_STATE_GRASP:
        if( m_p_robot_status->reachedPosition(m_p_stem_representation->getNearestXYZ()) )
        {
            m_state = STEMTRACK_STATE_FOLLOW;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> I have the stem, going to move up now");
        }
        break;

    case STEMTRACK_STATE_FOLLOW:
        if( reachedEndOfStem() )
        {
            m_state = STEMTRACK_STATE_END;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> I am done with my task");
        }
        break;
    }

    if(m_debug_state_par && m_state != state_prev)
        INFO_STREAM("In stemtrack monitor, state was: " << state_prev << " now set to: " << m_state << ".");
}

int StemTrackMonitor::getState()
{
    return m_state;
}

StemTrackMonitor::~StemTrackMonitor()
{
    //destructor
}
