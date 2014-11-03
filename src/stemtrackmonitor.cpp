#include "stemtrackmonitor.h"

StemTrackMonitor::StemTrackMonitor(StemRepresentation* p_stem_representation)
{
    m_p_stem_representation = p_stem_representation;
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

StemTrackMonitor::~StemTrackMonitor()
{
    //destructor
}
