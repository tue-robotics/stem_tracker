#ifndef STEMREPRESENTATION_H
#define STEMREPRESENTATION_H

#include <vector>
#include <iostream>

#include <ros/ros.h>

//Assumption: stem derivative is always positive

class StemRepresentation
{
    private:

        int m_stem_id, m_index_first_node_above;
        int m_n_nodes;
        std::vector<float> m_x_nodes;
        std::vector<float> m_y_nodes;
        std::vector<float> m_z_nodes;
        std::vector<float> m_nearestXYZ, m_start_at_xyz;
        std::vector<float> m_tangent_xyz;
        float m_stem_length_for_tangent, m_add_or_remove_node_euclidian_threshold;
        void updateNumberOfNodes();
        std::vector<float> getNodeXYZ(uint node);
        float m_max_stemtilt_xx, m_max_stemtilt_yy;

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
        inline const std::vector<float>& getTangent() { return m_tangent_xyz; }
        inline void setMaxTiltXX(float max) { m_max_stemtilt_xx = max; }
        inline void setMaxTiltYY(float max) { m_max_stemtilt_yy = max; }

        inline void setAddOrRemoveNodeThreshold( float threshold ) { m_add_or_remove_node_euclidian_threshold = threshold; }
        inline void setStemTrackingStartXYZ(std::vector<float> start_xyz) { m_start_at_xyz = start_xyz; }
        inline void setStemLengthToDetermineTangent(float lin_tan_d) { m_stem_length_for_tangent = lin_tan_d; }

        void updateStemNodes(const std::vector< std::vector<float> >& touches_xyz);
        void updateStemNodes(const std::vector<float>& touches_xyz, bool ignore_threshold = false);
        void updateStemNodes(const float& x, const float& y, const float& z, bool ignore_threshold);
        bool isXYZonStem(std::vector<float> xyz);
        void updateNearestXYZ(std::vector<float> from_xyz);
        void updateTangent();
        void loadNodesXYZ(std::vector<float> x, std::vector<float> y, std::vector<float> z);
        void flipNodes();
        void printAll();

        ~StemRepresentation();
};

#endif // STEMREPRESENTATION_H
