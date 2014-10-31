#ifndef VISUALIZATIONINTERFACE_H
#define VISUALIZATIONINTERFACE_H

#define INFO_STREAM     ROS_INFO_STREAM

#include "markerids.h"

class VisualizationInterface
{
    private:
        ros::Publisher* m_p_ros_marker_pub;
        std::string m_frame;
        std::string m_base_frame;
        std::vector<float> m_rgb;
        std::string m_name;
        int m_ros_marker_id;
        float m_sphere_radius;  // in meter
        float m_force_x;        // arrow diameter, in meter
        float m_force_y;        // arow head diameter, in meter
    public:
        VisualizationInterface(ros::Publisher* p_ros_marker_pub, std::string base_frame);
        bool configureSelf(MarkerIDs marker_id);
        void showXYZ(std::vector<float> xyz, MarkerIDs marker_id);
        void showXYZInRviz(std::vector<float> xyz);
        void showForce(std::vector<float> force, std::vector<float> origin, MarkerIDs marker_id);
        void showForceInRviz(std::vector<float> force, std::vector<float> origin);
};

#endif // VISUALIZATIONINTERFACE_H
