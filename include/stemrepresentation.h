#ifndef STEMREPRESENTATION_H
#define STEMREPRESENTATION_H

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

//Assumption: stem derivative is always positive

class StemRepresentation
{
    private:

        int m_stem_id;
        float m_rgb[3]; // between 0.0 - 1.0

        int m_n_nodes;
        std::vector<float> m_x_nodes;
        std::vector<float> m_y_nodes;
        std::vector<float> m_z_nodes;

        float m_thickness; // in meters

    public:
        StemRepresentation(int stem_id);
        std::vector<float> getStemXYZatZ(float z);
        bool selfCheck();
        void setRGB(float r, float g, float b);
        bool isXYZonStem(std::vector<float> xyz);
        void setThickness(float thickness);
        float getThickness();
        void addNodesXYZ(std::vector<float> x, std::vector<float> y, std::vector<float> z);
        int getNumberOfNodes();
        void flipNodes();
        void showInRviz(ros::Publisher* p_vis_pub);
        void printAll();

        ~StemRepresentation();
};

#endif // STEMREPRESENTATION_H
