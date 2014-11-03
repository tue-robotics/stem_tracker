#ifndef STEMTRACKMONITOR_H
#define STEMTRACKMONITOR_H

#include "stemrepresentation.h"

class StemTrackMonitor
{
private:
    StemRepresentation* m_p_stem_representation;
public:
    StemTrackMonitor(StemRepresentation* p_stem_representation);
    bool reachedEndOfStem(int up);
    ~StemTrackMonitor();
};

#endif // STEMTRACKMONITOR_H
