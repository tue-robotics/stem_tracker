#ifndef STEMREPRESENTATION_H
#define STEMREPRESENTATION_H

#include <vector>
#include <iostream>

#include <ros/ros.h>

//Assumption: stem derivative is always positive

class StemRepresentation
{
    private:

        int m_stem_id;
        int m_n_nodes;
        std::vector<float> m_x_nodes;
        std::vector<float> m_y_nodes;
        std::vector<float> m_z_nodes;
        std::vector<float> m_nearestXYZ;
        std::vector<float> m_xyz_below;
        std::vector<float> m_rot_xyz, m_start_at_xyz;
        float m_lin_tangent_d;

    public:
        StemRepresentation(int stem_id);

        void initializeTangent();

        std::vector<float> getStemXYZatZ(float z);

        inline const int getStemID() { return m_stem_id; }
        inline const int getNumberOfNodes() { return m_n_nodes; }
        inline const std::vector<float>& getNodesX() { return m_x_nodes; }
        inline const std::vector<float>& getNodesY() { return m_y_nodes; }
        inline const std::vector<float>& getNodesZ() { return m_z_nodes; }
        inline const std::vector<float>& getStemTrackingStartXYZ() const { return m_start_at_xyz; }
        inline const std::vector<float>& getNearestXYZ() { return m_nearestXYZ; }
        inline const std::vector<float>& getCurrentTangent() { return m_rot_xyz; }

        inline void setStemTrackingStartXYZ(std::vector<float> start_xyz) { m_start_at_xyz = start_xyz; }
        inline void setLinTangentDistance(float lin_tan_d) { m_lin_tangent_d = lin_tan_d; }

        bool isXYZonStem(std::vector<float> xyz);
        void updateNearestXYZ(std::vector<float> from_xyz);
        void updateLocalTangent();
        void updateXYZbelow();
        void loadNodesXYZ(std::vector<float> x, std::vector<float> y, std::vector<float> z);
        void flipNodes();
        void printAll();

        ~StemRepresentation();
};

#endif // STEMREPRESENTATION_H
