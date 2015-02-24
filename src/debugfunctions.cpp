#include "debugfunctions.h"
#include "loggingmacros.h"


void printKDLframe(KDL::Frame kdl_frame)
{
    std::stringstream frame_stream;

    INFO_STREAM("kdl frame:");

    for(int i=0; i<4; ++i)
    {
        frame_stream.str(""); frame_stream << std::scientific;

        for(int j=0; j<4; ++j)
            frame_stream << kdl_frame(i,j) << "\t";

        INFO_STREAM(frame_stream.str());
    }
}

void printKDLJntArray(KDL::JntArray kdl_array)
{
    if( kdl_array.rows() == 0)
    {
        WARNING_STREAM("Trying to print empty kdl array!");
        return;
    }

    INFO_STREAM("kdl joint array:");

    INFO_STREAM(std::scientific << kdl_array.data);

}

void printKDLVelframe(KDL::FrameVel kdl_vel_frame)
{
    std::stringstream frame_stream;

    KDL::Rotation rot = kdl_vel_frame.M.R;

    INFO_STREAM("kdl vel frame, Rotation:");
    for(int i=0; i<3; ++i)
    {
        frame_stream.str(""); frame_stream << std::scientific;

        for(int j=0; j<3; ++j)
            frame_stream << rot(i,j) << "\t";

        INFO_STREAM(frame_stream.str());
    }

    frame_stream.str(""); frame_stream << std::scientific;
    INFO_STREAM("kdl vel frame, Translation:");
    KDL::Vector vec = kdl_vel_frame.p.p;
    for(int i=0; i<3; ++i)
    {
        frame_stream << vec(i) << "\t";
    }
    INFO_STREAM(frame_stream.str());
}


void printVector(std::vector<float> vect)
{
    if(vect.size() == 0)
    {
        INFO_STREAM("vector empty");
        return;
    }

    std::stringstream tmp;
    for(uint i = 0; i < vect.size(); ++i)
        tmp << "vect[" << i << "] = " << vect.at(i) << "  ";

    INFO_STREAM(tmp.str());
    return;
}

void printVector(std::vector<bool> vect)
{
    if(vect.size() == 0)
    {
        INFO_STREAM("vector empty");
        return;
    }

    std::stringstream tmp;
    for(uint i = 0; i < vect.size(); ++i)
        tmp << "vect[" << i << "] = " << vect.at(i) << "  ";

    INFO_STREAM(tmp.str());
    return;
}

