#include "debugfunctions.h"

void printKDLframe(KDL::Frame kdl_frame)
{
    std::stringstream frame_stream;

    for(int i=0; i<4; ++i)
    {
        frame_stream.str(""); frame_stream << std::scientific;

        for(int j=0; j<4; ++j)
            frame_stream << kdl_frame(i,j) << "\t";

        INFO_STREAM(frame_stream.str());
    }
}

void printXYZvector(std::vector<float> vect)
{
    INFO_STREAM("x = " << vect.at(0) << " y = " << vect.at(1) << " z = " << vect.at(2));
}

