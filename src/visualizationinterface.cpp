#include "visualizationinterface.h"
#include "loggingmacros.h"

#include <math.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

void VisualizationInterface::connectToRos( const int &buffer_size)
{
    m_vis_marker_pub = m_node.advertise<visualization_msgs::Marker>( "stem_track_markers", buffer_size );
    m_vis_markerarray_pub = m_node.advertise<visualization_msgs::MarkerArray>( "stem_track_markerarray", buffer_size );
}

bool VisualizationInterface::configureSelf(const MarkerIDs& marker_id)
{

    switch(marker_id)
    {
    case gripper_center:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(1.0f);
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.0f);
        m_ros_marker_id = 0;
        m_sphere_radius = 0.015;
        m_name = "gripper_center";
        return true;

    case whisker_touch:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.0f);
        m_rgb.push_back(1.0f);
        m_ros_marker_id = 1;
        m_arrow_diam = 0.01;
        m_arrowhead_diam = 0.02;
        m_name = "whisker_touch";
        return true;

    case nearest_stem_intersection:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.0f);
        m_rgb.push_back(1.0f);
        m_rgb.push_back(0.0f);
        m_ros_marker_id = 2;
        m_sphere_radius = 0.015;
        m_name = "nearest_stem_intersection";
        return true;

    case stem:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.05f);
        m_rgb.push_back(0.65f);
        m_rgb.push_back(0.35f);
        m_linestrip_diam = 0.02;
        m_ros_marker_id = 3;
        m_name = "stem";
        return true;

    case stem_tangent:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(1.0f);
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.0f);
        m_ros_marker_id = 4;
        m_arrow_diam = 0.01;
        m_arrowhead_diam = 0.02;
        m_name = "stem_tangent";
        return true;

    default:
        WARNING_STREAM("Unknown marker id in visualization interface!");
        return false;
    }
}

void VisualizationInterface::showLineStrip(const std::vector<float>& x_coordinates, const std::vector<float>& y_coordinates, const std::vector<float>& z_coordinates, const MarkerIDs& marker_id)
{
    if(configureSelf(marker_id))
        showLineStripInRviz(x_coordinates, y_coordinates, z_coordinates);

    return;
}

void VisualizationInterface::showLineStripInRviz(const std::vector<float>& x_coordinates, const std::vector<float>& y_coordinates, const std::vector<float>& z_coordinates)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = m_frame;
    marker.header.stamp = ros::Time();

    marker.id = m_ros_marker_id;
    marker.ns = m_name;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = m_linestrip_diam;
    marker.color.a = 1.0f;
    marker.color.r = m_rgb.at(0);
    marker.color.g = m_rgb.at(1);
    marker.color.b = m_rgb.at(2);

    for(int i=0; i<x_coordinates.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = x_coordinates.at(i);
        p.y = y_coordinates.at(i);
        p.z = z_coordinates.at(i);
        marker.points.push_back(p);
    }

    m_vis_marker_pub.publish( marker );

}

void VisualizationInterface::showArrow(const std::vector<float>& xyz, const std::vector<float>& origin, const MarkerIDs& marker_id)
{
    if(xyz.size() != 3)
    {
        WARNING_STREAM("Unknown force vector in showArrow!");
        return;
    }

    if(origin.size() != 3)
    {
        WARNING_STREAM("Unknown origin vector in showArrow!");
        return;
    }

    if(configureSelf(marker_id))
        showArrowInRviz(xyz, origin);

    return;
}

void VisualizationInterface::showArrows(std::vector<float> angles, float offset, float length, const std::vector<float>& origin, const MarkerIDs& marker_id)
{
    if(origin.size() != 3)
    {
        WARNING_STREAM("Unknown origin vector in showArrow!");
        return;
    }

    for(uint i = 0; i < angles.size(); ++i)
    {
        if(angles[i]>360.0 || angles[i]<0.0)
        {
            WARNING_STREAM("Unknown angle in showArrow!");
            return;
        }
    }

    std::vector< std::vector<float> > xyz_s, origins;

    for(uint i = 0; i < angles.size(); ++i)
    {
        std::vector<float> xyz, origin_shifted; xyz.assign(3,0.0);
        float angle_rad = angles[i]/360.0f*2.0f*3.141592f;

        origin_shifted.push_back(origin[0] + cos(angle_rad)*(length+offset));
        origin_shifted.push_back(origin[1] + sin(angle_rad)*(length+offset));
        origin_shifted.push_back(origin[2]);

        xyz[0] = - cos(angle_rad)*length;
        xyz[1] = - sin(angle_rad)*length;

        xyz_s.push_back(xyz);
        origins.push_back(origin_shifted);
    }

    if(configureSelf(marker_id))
        showArrowsInRviz(xyz_s, origins);

    return;
}

void VisualizationInterface::showArrowsInRviz(const std::vector< std::vector<float> > & xyz, const std::vector< std::vector<float> >& origin)
{
    visualization_msgs::MarkerArray marker_array;
    for(uint i = 0; i < xyz.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = m_frame;
        marker.header.stamp = ros::Time().now();
        marker.id   = i + ARROW_IDS_OFFSET;
        addMarkerArrowId(marker.id);
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.ns = m_name;
        marker.scale.x = m_arrow_diam;
        marker.scale.y = m_arrowhead_diam;
        marker.color.a = 1.0f;
        marker.color.r = m_rgb.at(0);
        marker.color.g = m_rgb.at(1);
        marker.color.b = m_rgb.at(2);
        ros::Duration x_seconds(m_show_arrow_lifetime);
        marker.lifetime = x_seconds;

        /* construct nodes point */
        geometry_msgs::Point p_start, p_end;

        p_start.x = origin[i].at(0) - xyz[i].at(0);
        p_start.y = origin[i].at(1) - xyz[i].at(1);
        p_start.z = origin[i].at(2) - xyz[i].at(2);
        marker.points.push_back(p_start);

        p_end.x = origin[i].at(0);
        p_end.y = origin[i].at(1);
        p_end.z = origin[i].at(2);
        marker.points.push_back(p_end);

        marker_array.markers.push_back(marker);
    }
    m_vis_markerarray_pub.publish(marker_array);
}

void VisualizationInterface::showArrowInRviz(const std::vector<float>& xyz, const std::vector<float>& origin)
{
    /* construct line strip marker object */
    visualization_msgs::Marker marker;
    marker.header.frame_id = m_frame;
    marker.header.stamp = ros::Time().now();
    marker.id = m_ros_marker_id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = m_name;
    marker.scale.x = m_arrow_diam;
    marker.scale.y = m_arrowhead_diam;
    marker.color.a = 1.0f;
    marker.color.r = m_rgb.at(0);
    marker.color.g = m_rgb.at(1);
    marker.color.b = m_rgb.at(2);

    /* construct nodes point */
    geometry_msgs::Point p_start, p_end;

    p_start.x = origin.at(0) - xyz.at(0);
    p_start.y = origin.at(1) - xyz.at(1);
    p_start.z = origin.at(2) - xyz.at(2);
    marker.points.push_back(p_start);

    p_end.x = origin.at(0);
    p_end.y = origin.at(1);
    p_end.z = origin.at(2);
    marker.points.push_back(p_end);

    /* publish marker */
    m_vis_marker_pub.publish( marker );
}

void VisualizationInterface::showXYZ(const std::vector<float>& xyz, const MarkerIDs& marker_id)
{
    if(configureSelf(marker_id))
        showXYZInRviz(xyz);

    return;
}

void VisualizationInterface::showXYZInRviz(const std::vector<float>& xyz)
{

    visualization_msgs::Marker marker;

    marker.header.frame_id = m_frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = m_name;
    marker.action = visualization_msgs::Marker::ADD;

    marker.id = m_ros_marker_id;

    marker.pose.position.x = xyz.at(0);
    marker.pose.position.y = xyz.at(1);
    marker.pose.position.z = xyz.at(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = m_sphere_radius;
    marker.scale.y = m_sphere_radius;
    marker.scale.z = m_sphere_radius;

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.color.r = m_rgb.at(0);
    marker.color.g = m_rgb.at(1);
    marker.color.b = m_rgb.at(2);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    m_vis_marker_pub.publish( marker );

}

VisualizationInterface::~VisualizationInterface()
{
    m_vis_marker_pub.shutdown();
}
