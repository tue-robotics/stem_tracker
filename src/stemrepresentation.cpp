#include "stemrepresentation.h"
#include "loggingmacros.h"
#include <cmath>
#include "debugfunctions.h"

StemRepresentation::StemRepresentation(int stem_id=-1)
{
    m_stem_id = stem_id;
    m_n_nodes = 0;
    initializeTangent();
}

std::vector<float> StemRepresentation::getStemXYZatZ(float z)
{


    std::vector<float> xyz;
    xyz.clear();

    if (m_z_nodes.back() < z)
    {
        WARNING_STREAM("Z above stem length!");
        return xyz;
    }

    for( m_index_first_node_above=0; m_index_first_node_above<m_z_nodes.size(); ++m_index_first_node_above)
    {
        if(m_z_nodes.at(m_index_first_node_above) > z)
            break;
    }

    if (m_index_first_node_above < 1)
    {
        WARNING_STREAM("Z below stem start or not enough nodes to calc xy for stem at z!");
        return xyz;
    }

    float atFraction;
    if(m_z_nodes.at(m_index_first_node_above) - m_z_nodes.at(m_index_first_node_above-1) <= 0.0)
    {
        ERROR_STREAM("stem bends down or goes horizontally, we don't support that. sorry");
        atFraction = 0.0;
    }
    else
    {
        atFraction = (z - m_z_nodes.at(m_index_first_node_above-1)) / (m_z_nodes.at(m_index_first_node_above) - m_z_nodes.at(m_index_first_node_above-1));
    }

    xyz.push_back( m_x_nodes.at(m_index_first_node_above-1) + atFraction * ( m_x_nodes.at(m_index_first_node_above) - m_x_nodes.at(m_index_first_node_above-1) ) );
    xyz.push_back( m_y_nodes.at(m_index_first_node_above-1) + atFraction * ( m_y_nodes.at(m_index_first_node_above) - m_y_nodes.at(m_index_first_node_above-1) ) );
    xyz.push_back(z);

    return xyz;
}

void StemRepresentation::initializeTangent()
{
    m_tangent_xyz.clear();
    m_tangent_xyz.assign(3,0.0);
    m_tangent_bottom_xyz.assign(3,0.0);
}

std::vector<float> StemRepresentation::getNodeXYZ(uint node)
{
    std::vector<float> xyz;

    if(node > m_n_nodes -1 || node < 0)
    {
        ERROR_STREAM("Asking for node " << node << " which does not exist. N_nodes = " << m_n_nodes);
        return xyz;
    }

    xyz.push_back(m_x_nodes[node]);
    xyz.push_back(m_y_nodes[node]);
    xyz.push_back(m_z_nodes[node]);

    return xyz;
}

void StemRepresentation::updateTangent()
{
    initializeTangent();

    if((m_z_nodes.back() - m_z_nodes.front()) > m_lin_tangent_d)
    {
        m_tangent_bottom_xyz = getStemXYZatZ(m_z_nodes.back() - m_lin_tangent_d);
        uint n = m_z_nodes.size() - m_index_first_node_above;

        for(uint j = m_index_first_node_above; j < m_index_first_node_above + n; ++j)
        {
            /* get distance from regression starting point to this node */
            std::vector<float> dist;
            for(uint i = 0; i < 3; ++i)
                dist.push_back(getNodeXYZ(j)[i] - m_tangent_bottom_xyz[i]);

            /*normalize*/
            float len = sqrt(dist[0]*dist[0]+dist[1]*dist[1]+dist[2]*dist[2]);

            for(uint i = 0; i < 3; ++i)
                dist[i] /= len;

            /* add increment to overall tangent */
            for(uint i=0; i<3; ++i)
                m_tangent_xyz[i] +=  dist[i]/ ((float) n);

        }

        /* for plotting purposes, make stem tangent vector a bit smaller */
        for(uint i = 0; i < 3; ++i)
            m_tangent_xyz[i] /= ((float) 4);
    }
    else
    {
        WARNING_STREAM("Asking to update stem tangent while we have not enough stem");
    }

    return;

}

void StemRepresentation::updateNearestXYZ(std::vector<float> from_xyz)
{
    /* this function only used in simulation without gripper input */
    if(from_xyz.size() != 3)
    {
        WARNING_STREAM("Cannot updateNearestXYZ with input of length " << from_xyz.size() << ". I need xyz.");
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
        updateTangent();
    }
    return;
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

    return true;
}

void StemRepresentation::loadNodesXYZ(std::vector<float> x, std::vector<float> y, std::vector<float> z)
{

    m_x_nodes.clear();
    m_y_nodes.clear();
    m_z_nodes.clear();

    for( int i=0; i<x.size(); ++i )
    {
        m_x_nodes.push_back(x.at(i));
        m_y_nodes.push_back(y.at(i));
        m_z_nodes.push_back(z.at(i));
    }

    updateNumberOfNodes();
    return;
}

void StemRepresentation::flipNodes()
{
    for(int i=0; i<m_y_nodes.size(); ++i)
    {
        m_y_nodes.at(i) *= -1;
    }
}

void StemRepresentation::updateNumberOfNodes()
{
    if(m_x_nodes.size() != m_y_nodes.size() || m_y_nodes.size() != m_z_nodes.size())
    {
        ERROR_STREAM("In stemrepresentation x y z nodes don't match!");
        return;
    }
    m_n_nodes = m_x_nodes.size();
    return;
}

void StemRepresentation::updateStemNodes(const std::vector<float>& gripper_xyz, bool ignore_threshold)
{
    /* if gripper is not touched, assume stem is at middle of gripper */

    if(ignore_threshold)
    {
        m_x_nodes.push_back(gripper_xyz[0]);
        m_y_nodes.push_back(gripper_xyz[1]);
        m_z_nodes.push_back(gripper_xyz[2]);
    }
    else
    {
        std::vector< std::vector<float> > possible_stem_xyz_s;
        possible_stem_xyz_s.push_back(gripper_xyz);
        updateStemNodes(possible_stem_xyz_s);
    }
    return;
}

void StemRepresentation::updateStemNodes(const std::vector< std::vector<float> >& possible_stem_xyz_s)
{
    /* first: average xyz of touched whiskers, possibly the
        gripper is touched at multiple places at once*/
    std::vector<float> average_point_of_touch;
    average_point_of_touch.assign(3,0.0);
    for(uint i = 0; i < possible_stem_xyz_s.size(); ++i)
    {
        for(uint j = 0; j < 3; ++j)
            average_point_of_touch[j] += possible_stem_xyz_s[i][j] / ((float) possible_stem_xyz_s.size());
    }

    /* check distance to highest node of stem */
    float dist = sqrt(  pow(average_point_of_touch[0]-m_x_nodes.back(),2.0) +
                        pow(average_point_of_touch[1]-m_y_nodes.back(),2.0) +
                        pow(average_point_of_touch[2]-m_z_nodes.back(),2.0)  );

    if(dist > m_add_or_remove_node_euclidian_threshold)
    {
        if(average_point_of_touch[2] > m_z_nodes.back())
        {
            /* add a stem node */
            m_x_nodes.push_back(average_point_of_touch[0]);
            m_y_nodes.push_back(average_point_of_touch[1]);
            m_z_nodes.push_back(average_point_of_touch[2]);

        }
        else if (m_n_nodes > 2)
        {
            /* remove a stem node */
            m_x_nodes.pop_back();
            m_y_nodes.pop_back();
            m_z_nodes.pop_back();
        }

        updateNumberOfNodes();
    }

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
