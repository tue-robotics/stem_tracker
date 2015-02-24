#ifndef VISUALIZATIONINTERFACE_H
#define VISUALIZATIONINTERFACE_H

#include <vector>
#include <string>

#include <ros/ros.h>

enum MarkerIDs
{
    stem,
    nearest_stem_intersection,
    gripper_center,
    whisker_touch,
    stem_tangent
};

#define ARROW_IDS_OFFSET 10;

class VisualizationInterface
{
private:
    ros::NodeHandle m_node;
    ros::Publisher m_vis_marker_pub, m_vis_markerarray_pub;
    std::string m_frame;
    std::string m_base_frame;
    std::vector<float> m_rgb;
    std::string m_name;
    float m_show_arrow_lifetime;
    int m_ros_marker_id;
    float m_sphere_radius;     // in meters
    float m_linestrip_diam;    // in meters
    float m_arrow_diam;        // arrow diameter, in meters
    float m_arrowhead_diam;    // arow head diameter, in meters
public:
    VisualizationInterface(ros::NodeHandle node, std::string base_frame) : m_node(node), m_base_frame(base_frame) {}

    void connectToRos(const int& buffer_size);
    inline setShowArrowLifetime(float seconds) { m_show_arrow_lifetime = seconds; }
    bool configureSelf(const MarkerIDs& marker_id);
    void showLineStrip(const std::vector<float>& x_coordinates, const std::vector<float>& y_coordinates, const std::vector<float>& z_coordinates, const MarkerIDs& marker_id);
    void showLineStripInRviz(const std::vector<float>& x_coordinates, const std::vector<float>& y_coordinates, const std::vector<float>& z_coordinates);
    void showXYZ(const std::vector<float>& xyz, const MarkerIDs& marker_id);
    void showXYZInRviz(const std::vector<float>& xyz);
    void showArrow(const std::vector<float>& force, const std::vector<float>& origin, const MarkerIDs& marker_id);
    void showArrows(std::vector<float> angle, float offset, float length, const std::vector<float>& origin, const MarkerIDs& marker_id);
    void showArrowInRviz(const std::vector<float>& force, const std::vector<float>& origin);
    void showArrowsInRviz(const std::vector< std::vector<float> > & xyz, const std::vector< std::vector<float> >& origin);

    virtual ~VisualizationInterface();
};

#endif // VISUALIZATIONINTERFACE_H
