#ifndef STEMTRACKMONITOR_H
#define STEMTRACKMONITOR_H

#include "stemrepresentation.h"

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
    int m_state;
public:
    StemTrackMonitor(StemRepresentation* p_stem_representation);
    bool reachedEndOfStem(int up);
    void updateState();
    int getState();
    ~StemTrackMonitor();
};

#endif // STEMTRACKMONITOR_H
