#include "stemrepresentation.h"

StemRepresentation::StemRepresentation(int stem_id=-1)
{
    m_stem_id = stem_id;
    m_n_nodes = 0;
    m_thickness = -1.0;
    m_rgb[0] = -1.0;
    m_rgb[1] = -1.0;
    m_rgb[2] = -1.0;
}

std::vector<float> StemRepresentation::getStemXYZatZ(float z)
{

    std::vector<float> xyz;

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

bool StemRepresentation::selfCheck()
{

    bool IamOK = true;

    if( !( ( m_x_nodes.size() == m_y_nodes.size() ) && ( m_y_nodes.size() == m_z_nodes.size() ) ) )
    {
        INFO_STREAM("in stem with id " << m_stem_id << ", vectors with node coordinates not of equal length!");
        INFO_STREAM("\t x_nodes.size() = " << m_x_nodes.size() << " y_nodes.size() = " << m_y_nodes.size() << " z_nodes.size() = " << m_z_nodes.size() << std::endl);
        IamOK = false;
    }

    for(int i=0;i<3;++i)
    {
        if(m_rgb[i] < 0.0)
        {
            INFO_STREAM("In stem with id " << m_stem_id << ", rgb value not initialized or set to value smaller than 0.0");
            IamOK = false;
        }
    }

    if(m_thickness <= 0.0)
    {
        INFO_STREAM("In stem with id " << m_stem_id << ", thickness not initialized or set to value smaller than 0.0");
        IamOK = false;
    }

    return IamOK;
}

void StemRepresentation::setRGB(float r, float g, float b)
{
    m_rgb[0] = r;
    m_rgb[1] = g;
    m_rgb[2] = b;
}

void StemRepresentation::setFrame(const std::string frame)
{
    m_frame = frame;
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

void StemRepresentation::setThickness(float thickness)
{
    m_thickness = thickness;
}

float StemRepresentation::getThickness()
{
    return m_thickness;
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

void StemRepresentation::showInRviz(ros::Publisher* p_vis_pub, const std::string ns)
{
    if(!selfCheck())
        return;

    /* construct line strip marker object */
    visualization_msgs::Marker marker;
    marker.header.frame_id = m_frame;
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.ns = ns;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = m_thickness;
    marker.color.a = 1.0;
    marker.color.r = m_rgb[0];
    marker.color.g = m_rgb[1];
    marker.color.b = m_rgb[2];

    /* construct nodes point */
    for(int i=0; i<m_x_nodes.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = m_x_nodes.at(i);
        p.y = m_y_nodes.at(i);
        p.z = m_z_nodes.at(i);
        marker.points.push_back(p);
    }

    /* publish marker */
    p_vis_pub->publish( marker );

}

void StemRepresentation::printAll()
{

    INFO_STREAM("===============");

    INFO_STREAM("Stem id: " << m_stem_id);

    INFO_STREAM("RGB: " << m_rgb[0] << " " << m_rgb[1] << " " << m_rgb[2] << " ");

    INFO_STREAM("Thickness: " << m_thickness << " meters");

    INFO_STREAM("Number of nodes: " << m_n_nodes);

    std::stringstream nodes_stream;
    nodes_stream << "Nodes:" << std::endl;

    for(int i=0;i<m_n_nodes;++i)
        nodes_stream << "\t\t\t\t\t" << m_x_nodes.at(i) << "\t" << m_y_nodes.at(i) << "\t" << m_z_nodes.at(i) << std::endl;

    INFO_STREAM(nodes_stream.str());

    INFO_STREAM("===============");

}

StemRepresentation::~StemRepresentation()
{
    // destructor
}
