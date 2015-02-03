#include "stemtrackmonitor.h"
#include "loggingmacros.h"

#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "stemtrackcontroller.h"
#include "whiskerinterpreter.h"

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

const std::string StemTrackMonitor::stateToString(stemtrack_state_t state) const
{
    switch( state )
    {
    case INIT:
        return "INIT";
    case PREPOS:
        return "PREPOS";
    case GRASP:
        return "GRASP";
    case FOLLOW:
        return "FOLLOW";
    case LOST:
        return "LOST";
    case END:
        return "END";
    case SIDEBRANCH:
        return "SIDEBRANCH";
    case ERROR:
        return "ERROR";
    default:
        WARNING_STREAM("Unknown state in StemTrackMonitor::stateToString!");
        return "UNKNOWN STATE!";
    }
}

void StemTrackMonitor::updateState()
{
    stemtrack_state_t state_prev = m_state;

    switch (m_state)
    {

    case INIT:
        if( m_p_whisker_interpreter->isInitialized())
        {
            m_state = PREPOS;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> Obtained nominal whisker values, going to my preposition now");
        }
        break;

    case PREPOS:
        if( m_p_robot_status->reachedPosition( m_p_robot_representation->getInitialPoseJointRefs() ) )
        {
            m_state = GRASP;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> I am going to grasp the stem, hihaa");
        }
        break;

    case GRASP:
        if( m_p_robot_status->reachedPosition(m_p_stem_representation->getNearestXYZ()) )
        {
            m_state = FOLLOW;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> I have the stem, going to move up now");
        }
        break;

    case FOLLOW:
        if( reachedEndOfStem() )
        {
            m_state = END;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> I am done with my task");
        }
        break;
    }

    if(m_debug_state_par && m_state != state_prev)
        INFO_STREAM("In stemtrack monitor, state was: " << stateToString(state_prev) << " now set to: " << stateToString(m_state) << ".");
}

StemTrackMonitor::~StemTrackMonitor()
{
    //destructor
}
