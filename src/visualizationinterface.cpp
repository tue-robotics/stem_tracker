#include "visualizationinterface.h"
#include "loggingmacros.h"
#include "debugfunctions.h"

#include <math.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Duration VisualizationInterface::getRosDuration(float seconds)
{
    ros::Duration sec_dur(seconds);
    return sec_dur;
}

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
        m_ros_marker_id = 1;
        m_sphere_radius = 0.015;
        m_name = "gripper_center";
        m_lifetime = m_show_setpoint_lifetime;

        return true;

    case cartesian_setpoint:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.0f);
        m_rgb.push_back(1.0f);
        m_rgb.push_back(0.0f);
        m_ros_marker_id = 2;
        m_sphere_radius = 0.015;
        m_lifetime = m_show_setpoint_lifetime;
        m_name = "cartesian_setpoint";
        return true;

    case white_debug_dot:
        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(1.0f);
        m_rgb.push_back(1.0f);
        m_rgb.push_back(1.0f);
        m_ros_marker_id = 3;
        m_sphere_radius = 0.015;
        m_lifetime = m_show_setpoint_lifetime;
        m_name = "white_debug_dot";
        return true;

    case whisker_touch:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.0f);
        m_rgb.push_back(1.0f);
        m_ros_marker_id = 4;
        m_arrow_diam = 0.01;
        m_arrowhead_diam = 0.02;
        m_name = "whisker_touch";
        m_lifetime = m_show_whisker_arrow_lifetime;

        return true;

    case stem:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.05f);
        m_rgb.push_back(0.65f);
        m_rgb.push_back(0.35f);
        m_linestrip_diam = 0.02;
        m_lifetime = -1.0;
        m_ros_marker_id = 5;
        m_name = "stem";
        return true;

    case stem_tangent:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.5f);
        m_rgb.push_back(0.5f);
        m_ros_marker_id = 6;
        m_arrow_diam = 0.01;
        m_arrowhead_diam = 0.02;
        m_multiply_arrow_with = m_stem_tangent_arrow_multiplication;
        m_lifetime = m_show_stem_tangent_lifetime;
        m_name = "stem_tangent";
        return true;

    case red_debug_arrow:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(1.0f);
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.0f);
        m_ros_marker_id = 7;
        m_arrow_diam = 0.01;
        m_arrowhead_diam = 0.02;
        m_multiply_arrow_with = m_debug_arrow_multiplication;
        m_lifetime = m_debug_arrow_lifetime;
        m_name = "red_debug_arrow";
        return true;

    case green_debug_arrow:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.0f);
        m_rgb.push_back(1.0f);
        m_rgb.push_back(0.0f);
        m_ros_marker_id = 8;
        m_arrow_diam = 0.01;
        m_arrowhead_diam = 0.02;
        m_multiply_arrow_with = m_debug_arrow_multiplication;
        m_lifetime = m_debug_arrow_lifetime;
        m_name = "green_debug_arrow";
        return true;

    case blue_debug_arrow:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.0f);
        m_rgb.push_back(1.0f);
        m_ros_marker_id = 9;
        m_arrow_diam = 0.01;
        m_arrowhead_diam = 0.02;
        m_multiply_arrow_with = m_debug_arrow_multiplication;
        m_lifetime = m_debug_arrow_lifetime;
        m_name = "blue_debug_arrow";
        return true;

    case tomato_truss:
        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(1.0f);
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.0f);
        m_ros_marker_id = 10;
        m_sphere_radius = 0.05;
        m_lifetime = m_show_setpoint_lifetime;
        m_name = "tomato_truss";
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

    if(m_lifetime >= 0.0)
        marker.lifetime = getRosDuration(m_lifetime);

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

void VisualizationInterface::showArrow(const std::vector<float>& xyz, const float& origin_x, const float& origin_y, const float& origin_z, const MarkerIDs& marker_id)
{
    std::vector<float> origin;
    origin.push_back(origin_x);
    origin.push_back(origin_y);
    origin.push_back(origin_z);
    showArrow(xyz, origin, marker_id);
    return;
}

void VisualizationInterface::showArrow(const std::vector<float>& xyz, const std::vector<float>& origin, const MarkerIDs& marker_id)
{
    if(xyz.size() != 3)
    {
        WARNING_STREAM("Unknown xyz vector in showArrow!");
        return;
    }

    if(origin.size() != 3)
    {
        WARNING_STREAM("Unknown origin vector in showArrow!");
        return;
    }

    if(configureSelf(marker_id))
    {
        std::vector<float> xyz_ = xyz;
        for(uint i = 0; i < 3; ++i)
            xyz_[i] *= m_multiply_arrow_with;

        showArrowInRviz(xyz_, origin);
    }

    return;
}

void VisualizationInterface::showArrows(const std::vector< std::vector<float> >& tips, const std::vector< std::vector<float> >& origins, const MarkerIDs& marker_id)
{
    if(configureSelf(marker_id))
        showArrowsInRviz(tips, origins);

    return;
}

void VisualizationInterface::showArrowsInRviz(const std::vector< std::vector<float> >& tips, const std::vector< std::vector<float> >& origins)
{
    visualization_msgs::MarkerArray marker_array;
    for(uint i = 0; i < tips.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = m_frame;
        marker.header.stamp = ros::Time().now();
        marker.id   = i + ARROW_IDS_OFFSET;
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

        if(m_lifetime >= 0.0)
            marker.lifetime = getRosDuration(m_lifetime);

        /* construct nodes point */
        geometry_msgs::Point p_start, p_end;

        p_start.x = origins[i].at(0);
        p_start.y = origins[i].at(1);
        p_start.z = origins[i].at(2);
        marker.points.push_back(p_start);

        p_end.x = tips[i].at(0);
        p_end.y = tips[i].at(1);
        p_end.z = tips[i].at(2);
        marker.points.push_back(p_end);

        marker_array.markers.push_back(marker);
    }
    m_vis_markerarray_pub.publish(marker_array);
}

void VisualizationInterface::showArrowInRviz(const std::vector<float>& tip, const std::vector<float>& origin)
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

    if(m_lifetime >= 0.0)
        marker.lifetime = getRosDuration(m_lifetime);

    /* construct nodes point */
    geometry_msgs::Point p_start, p_end;

    p_start.x = origin.at(0);
    p_start.y = origin.at(1);
    p_start.z = origin.at(2);
    marker.points.push_back(p_start);

    p_end.x = origin.at(0) + tip.at(0);
    p_end.y = origin.at(1) + tip.at(1);
    p_end.z = origin.at(2) + tip.at(2);
    marker.points.push_back(p_end);

    /* publish marker */
    m_vis_marker_pub.publish( marker );
}
void VisualizationInterface::showXYZ(const std::vector<float>& xyz, const std::vector<float> xyz_2, const MarkerIDs& marker_id)
{
    std::vector<float> sum;
    sum.assign(3,0.0);
    for(uint i = 0; i < 3; ++i)
        sum[i] = xyz[i] + xyz_2[i];
    showXYZ(sum, marker_id);
    return;
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

    if(m_lifetime >= 0.0)
        marker.lifetime = getRosDuration(m_lifetime);

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.color.r = m_rgb.at(0);
    marker.color.g = m_rgb.at(1);
    marker.color.b = m_rgb.at(2);
    marker.color.a = 1.0;

    m_vis_marker_pub.publish( marker );

}

VisualizationInterface::~VisualizationInterface()
{
    m_vis_marker_pub.shutdown();
}
