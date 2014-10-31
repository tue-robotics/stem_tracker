#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "visualizationinterface.h"

VisualizationInterface::VisualizationInterface(ros::Publisher* p_ros_marker_pub, std::string base_frame)
{
    m_p_ros_marker_pub = p_ros_marker_pub;
    m_base_frame = base_frame;
}

void VisualizationInterface::configureSelf(MarkerIDs marker_id)
{

    switch(marker_id)
    {
        case gripper_center:

            m_frame = m_base_frame;
            m_rgb.clear();
            m_rgb.push_back(1.0f);
            m_rgb.push_back(0.0f);
            m_rgb.push_back(0.0f);
            m_ros_marker_id = 2;
            m_sphere_radius = 0.025;
            m_name = "gripper_center";
            break;

        default:
            INFO_STREAM("Unknown marker id in visualization interface!");
    }
}

void VisualizationInterface::showXYZ(std::vector<float> xyz, MarkerIDs marker_id)
{
    configureSelf(marker_id);
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

    m_p_ros_marker_pub->publish( marker );

}
