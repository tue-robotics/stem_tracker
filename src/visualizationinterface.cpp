#include "visualizationinterface.h"

VisualizationInterface::VisualizationInterface(ros::NodeHandle node, std::string base_frame)
{
    m_node = node;
    m_vis_pub = m_node.advertise<visualization_msgs::Marker>( "stem_track_markers", 100 );
    m_base_frame = base_frame;
}

bool VisualizationInterface::configureSelf(MarkerIDs marker_id)
{

    switch(marker_id)
    {
    case gripper_center:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(1.0f);
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.0f);
        m_ros_marker_id = marker_id;
        m_sphere_radius = 0.015;
        m_name = "gripper_center";
        return true;

    case whisker_net_force:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.0f);
        m_rgb.push_back(0.0f);
        m_rgb.push_back(1.0f);
        m_ros_marker_id = marker_id;
        m_arrow_diam = 0.01;
        m_arrowhead_diam = 0.02;
        m_name = "whisker_force";
        return true;

    case nearest_stem_intersection:

        m_frame = m_base_frame;
        m_rgb.clear();
        m_rgb.push_back(0.0f);
        m_rgb.push_back(1.0f);
        m_rgb.push_back(0.0f);
        m_ros_marker_id = marker_id;
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
        m_ros_marker_id = marker_id;
        m_name = "stem";
        return true;

    default:
        INFO_STREAM("Unknown marker id in visualization interface!");
        return false;
    }
}

void VisualizationInterface::showLineStrip(std::vector<float> x_coordinates, std::vector<float> y_coordinates, std::vector<float> z_coordinates, MarkerIDs marker_id)
{
    if(configureSelf(marker_id))
        showLineStripInRviz(x_coordinates, y_coordinates, z_coordinates);

    return;
}

void VisualizationInterface::showLineStripInRviz(std::vector<float> x_coordinates, std::vector<float> y_coordinates, std::vector<float> z_coordinates)
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

    m_vis_pub.publish( marker );

}

void VisualizationInterface::showForce(std::vector<float> force, std::vector<float> origin, MarkerIDs marker_id)
{
    if(force.size() != 2)
    {
        INFO_STREAM("unknown force vector in showforce");
        return;
    }

    if(origin.size() != 3)
    {
        INFO_STREAM("unknown origin vector in showforce");
        return;
    }

    if(configureSelf(marker_id))
        showForceInRviz(force, origin);

    return;
}

void VisualizationInterface::showForceInRviz(std::vector<float> force, std::vector<float> origin)
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

    p_start.x = origin.at(0) - force.at(0);
    p_start.y = origin.at(1) - force.at(1);
    p_start.z = origin.at(2);
    marker.points.push_back(p_start);

    p_end.x = origin.at(0);
    p_end.y = origin.at(1);
    p_end.z = origin.at(2);
    marker.points.push_back(p_end);

    /* publish marker */
    m_vis_pub.publish( marker );
}

void VisualizationInterface::showXYZ(std::vector<float> xyz, MarkerIDs marker_id)
{
    if(configureSelf(marker_id))
        showXYZInRviz(xyz);

    return;
}

void VisualizationInterface::showXYZInRviz(std::vector<float> xyz)
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

    m_vis_pub.publish( marker );

}
