#ifndef VISUALIZATIONINTERFACE_H
#define VISUALIZATIONINTERFACE_H

#include <vector>
#include <string>

#include <ros/ros.h>

enum MarkerIDs
{
    stem,
    cartesian_setpoint,
    gripper_center,
    whisker_touch,
    stem_tangent,
    red_debug_arrow,
    green_debug_arrow,
    blue_debug_arrow,
    white_debug_dot,
    peduncle,
    tomato_1,
    tomato_2,
    tomato_3,
    tomato_4,
    tomato_5
};

#define ARROW_IDS_OFFSET 10;

class VisualizationInterface
{
private:
    ros::Duration getRosDuration(float seconds);

    ros::NodeHandle m_node;
    ros::Publisher m_vis_marker_pub, m_vis_markerarray_pub;
    std::string m_frame;
    std::string m_base_frame;
    std::vector<float> m_rgb;
    std::string m_name;
    float m_show_whisker_arrow_lifetime, m_lifetime, m_show_setpoint_lifetime, m_show_stem_tangent_lifetime, m_debug_arrow_lifetime;
    int m_ros_marker_id;
    float m_sphere_radius;     // in meters
    float m_linestrip_diam;    // in meters
    float m_arrow_diam;        // arrow diameter, in meters
    float m_arrowhead_diam;    // arow head diameter, in meters
    bool m_truss_in_wrist_than_show_left;
    float m_multiply_arrow_with, m_stem_tangent_arrow_multiplication, m_debug_arrow_multiplication;
public:
    VisualizationInterface(ros::NodeHandle node, std::string base_frame) : m_node(node), m_base_frame(base_frame) {}

    void connectToRos(const int& buffer_size);

    inline void setShowWhiskerArrowLifetime(float seconds) { m_show_whisker_arrow_lifetime = seconds; }
    inline void setShowSetPointLifetime(float seconds) { m_show_setpoint_lifetime = seconds; }
    inline void setShowStemTangentLifetime(float seconds) { m_show_stem_tangent_lifetime = seconds; }
    inline void setStemTangentArrowElongation(float multiply_with) { m_stem_tangent_arrow_multiplication = multiply_with; }
    inline void setDebugArrowElongation(float multiply_with) { m_debug_arrow_multiplication = multiply_with; }
    inline void setDebugArrowLifetime(float lifetime) { m_debug_arrow_lifetime = lifetime; }
    inline void setTrussInWristThanShowLeft(bool show_left) { m_truss_in_wrist_than_show_left = show_left; }

    inline const float& getStemTangentVectorElongation() const { return m_stem_tangent_arrow_multiplication; }

    bool configureSelf(const MarkerIDs& marker_id);
    void showLineStrip(const std::vector<float>& x_coordinates, const std::vector<float>& y_coordinates, const std::vector<float>& z_coordinates, const MarkerIDs& marker_id);
    void showLineStripInRviz(const std::vector<float>& x_coordinates, const std::vector<float>& y_coordinates, const std::vector<float>& z_coordinates);
    void showXYZ(const std::vector<float>& xyz, const MarkerIDs& marker_id);
    void showXYZ(const std::vector<float>& xyz, const std::vector<float> xyz_2, const MarkerIDs& marker_id);
    void showXYZInRviz(const std::vector<float>& xyz);
    void showArrow(const std::vector<float>& xyz, const std::vector<float>& origin, const MarkerIDs& marker_id);
    void showArrow(const std::vector<float>& xyz, const float& origin_x, const float& origin_y, const float& origin_z, const MarkerIDs& marker_id);
    void showArrows(const std::vector< std::vector<float> >& tips, const std::vector< std::vector<float> >& origins, const MarkerIDs& marker_id);
    void showArrowInRviz(const std::vector<float>& tip, const std::vector<float>& origin);
    void showArrowsInRviz(const std::vector< std::vector<float> >& tips, const std::vector< std::vector<float> >& origins);
    void showTomatoTruss(const float& angle, const std::vector<float>& origin);

    virtual ~VisualizationInterface();
};

#endif // VISUALIZATIONINTERFACE_H
