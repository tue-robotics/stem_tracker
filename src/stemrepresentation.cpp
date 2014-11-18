#include "stemrepresentation.h"

StemRepresentation::StemRepresentation(int stem_id=-1)
{
    m_stem_id = stem_id;
    m_n_nodes = 0;
    initializeTangent();
}

int StemRepresentation::getStemID()
{
    return m_stem_id;
}

std::vector<float> StemRepresentation::getStemXYZatZ(float z)
{

    std::vector<float> xyz;
    xyz.clear();

    if (m_z_nodes.back() < z)
    {
        INFO_STREAM("z above stem length!");
        return xyz;
    }

    int index_first_above;
    for( index_first_above=0; index_first_above<m_z_nodes.size(); ++index_first_above)
    {
        if(m_z_nodes.at(index_first_above) > z)
        {
            break;
        }
    }

    if (index_first_above < 1)
    {
        INFO_STREAM("z below stem start or not enough nodes to calc xy for stem at z!");
        return xyz;
    }

    float atFraction;
    if(m_z_nodes.at(index_first_above) - m_z_nodes.at(index_first_above-1) <= 0.0)
    {
        INFO_STREAM("stem bends down or goes horizontally, we don't support that. sorry");
        atFraction = 0.0;
    }
    else
    {
        atFraction = (z - m_z_nodes.at(index_first_above-1)) / (m_z_nodes.at(index_first_above) - m_z_nodes.at(index_first_above-1));
    }

    xyz.push_back( m_x_nodes.at(index_first_above-1) + atFraction * ( m_x_nodes.at(index_first_above) - m_x_nodes.at(index_first_above-1) ) );
    xyz.push_back( m_y_nodes.at(index_first_above-1) + atFraction * ( m_y_nodes.at(index_first_above) - m_y_nodes.at(index_first_above-1) ) );
    xyz.push_back(z);

    return xyz;

}

void StemRepresentation::setLinTangentDistance(float lin_tan_d)
{
    m_lin_tangent_d = lin_tan_d;
}

void StemRepresentation::initializeTangent()
{
    m_rot_xyz.clear();
    m_rot_xyz.assign(3,0.0);
}

std::vector<float> StemRepresentation::getCurrentTangent()
{
    return m_rot_xyz;
}

void StemRepresentation::updateLocalTangent()
{
    if(m_nearestXYZ.size() != 3)
    {
        INFO_STREAM("trying to find tangent while nearest stem xyz is not known!");
        return;
    }

    updateXYZbelow();
    if(m_xyz_below.size() == 3)
    {
        if(m_nearestXYZ.at(2) - m_xyz_below.at(2) > m_lin_tangent_d)
        {
            for(int i=0; i<3; ++i)
                m_rot_xyz.at(i) = m_nearestXYZ.at(i) - m_xyz_below.at(i);
        }
    }
    else
        initializeTangent();
}

void StemRepresentation::updateXYZbelow()
{
    if(m_nearestXYZ.at(2)-m_lin_tangent_d > m_z_nodes.front())
        m_xyz_below = getStemXYZatZ(m_nearestXYZ.at(2)-m_lin_tangent_d);
}

void StemRepresentation::updateNearestXYZ(std::vector<float> from_xyz)
{
    if(from_xyz.size() != 3)
    {
        INFO_STREAM("cannot updateNearestXYZ with input of length " << from_xyz.size());
        return;
    }

    m_nearestXYZ.clear();

    if (m_z_nodes.back() < from_xyz.at(2))
    {
        m_nearestXYZ.push_back(m_x_nodes.back());
        m_nearestXYZ.push_back(m_y_nodes.back());
        m_nearestXYZ.push_back(m_z_nodes.back());
        initializeTangent();
        return;
    }
    else if (m_z_nodes.front() > from_xyz.at(2))
    {
        m_nearestXYZ.push_back(m_x_nodes.front());
        m_nearestXYZ.push_back(m_y_nodes.front());
        m_nearestXYZ.push_back(m_z_nodes.front());
        initializeTangent();
        return;
    }
    else
    {
        m_nearestXYZ = getStemXYZatZ(from_xyz.at(2));
        updateLocalTangent();
    }
    return;
}

std::vector<float> StemRepresentation::getNearestXYZ()
{
    return m_nearestXYZ;
}

bool StemRepresentation::selfCheck()
{

    bool IamOK = true;

    if( !( ( m_x_nodes.size() == m_y_nodes.size() ) && ( m_y_nodes.size() == m_z_nodes.size() ) ) )
    {
        INFO_STREAM("in stem with id " << m_stem_id << ", vectors with node coordinates not of equal length!");
        INFO_STREAM("\t x_nodes.size() = " << m_x_nodes.size() << " y_nodes.size() = " << m_y_nodes.size() << " z_nodes.size() = " << m_z_nodes.size() << std::endl);
        IamOK = false;
    }

    return IamOK;
}

bool StemRepresentation::isXYZonStem(std::vector<float> xyz)
{
    if(xyz.size() != 3)
    {
        return false;
    }

    if(m_z_nodes.back() < xyz[2]  || m_z_nodes.front() > xyz[2] )
    {
        return false;
    }

    //todo: check if really on stem

    return true;
}

void StemRepresentation::loadNodesXYZ(std::vector<float> x, std::vector<float> y, std::vector<float> z)
{
    if(!selfCheck())
        return;

    m_x_nodes.clear();
    m_y_nodes.clear();
    m_z_nodes.clear();

    for( int i=0; i<x.size(); ++i )
    {
        m_x_nodes.push_back(x.at(i));
        m_y_nodes.push_back(y.at(i));
        m_z_nodes.push_back(z.at(i));
    }

    m_n_nodes = m_x_nodes.size();

    selfCheck();

}

int StemRepresentation::getNumberOfNodes()
{
    return m_n_nodes;
}

void StemRepresentation::flipNodes()
{
    for(int i=0; i<m_y_nodes.size(); ++i)
    {
        m_y_nodes.at(i) *= -1;
    }
}

std::vector<float> StemRepresentation::getNodesX()
{
    return m_x_nodes;
}

std::vector<float> StemRepresentation::getNodesY()
{
    return m_y_nodes;
}

std::vector<float> StemRepresentation::getNodesZ()
{
    return m_z_nodes;
}

void StemRepresentation::printAll()
{

    INFO_STREAM("Stem id: " << m_stem_id);

    INFO_STREAM("Number of nodes: " << m_n_nodes);

    std::stringstream nodes_stream;
    nodes_stream << "Nodes:" << std::endl;

    for(int i=0;i<m_n_nodes;++i)
        nodes_stream << "\t\t\t\t\t" << m_x_nodes.at(i) << "\t" << m_y_nodes.at(i) << "\t" << m_z_nodes.at(i) << std::endl;

    INFO_STREAM(nodes_stream.str());

}

StemRepresentation::~StemRepresentation()
{
    // destructor
}
