#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H

/* c++ includes */
#include <vector>
#include <map>
#include <iostream>

/* ros includes */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>

/* kdl includes */
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
//#include <kdl/treejnttojacsolver.hpp>

/* amigo includes */
#include <profiling/StatsPublisher.h>


class VirtualStem
{

    /* Nodes have to be specified as a sequence
     * of floats. Number of floats per node can be anything
     * larger than zero.
     *
     * This class depends on:
        <vector>
        <iostream>
        <ros/ros.h>
        <visualization_msgs/Marker.h>
    */

    private:

        int m_stem_id;
        float m_rgb[3]; // between 0.0 - 1.0
        int m_n_nodes;
        int m_node_dimension;
        float m_thickness; // in cm
        std::vector<float> m_nodes;

    public:

        VirtualStem(int stem_id=-1){
            m_stem_id = stem_id;
            m_n_nodes = 0;
            m_node_dimension = -1;
            m_thickness = -1.0;
            m_rgb[0] = -1.0;
            m_rgb[1] = -1.0;
            m_rgb[2] = -1.0;
        }

        bool selfCheck(){

            bool IamOK = true;

            if(m_node_dimension < 1){
                INFO_STREAM("In stem with id " << m_stem_id << ", node dimension was not initialized or was set to value smaller than 1");
                IamOK = false;
            }

            for(int i=0;i<3;++i){
                if(m_rgb[i] < 0.0){
                    INFO_STREAM("In stem with id " << m_stem_id << ", rgb value not initialized or set to value smaller than 0.0");
                    IamOK = false;
                }
            }

            if(m_thickness <= 0.0){
                INFO_STREAM("In stem with id " << m_stem_id << ", thickness not initialized or set to value smaller than 0.0");
                IamOK = false;
            }

            return IamOK;
        }

        void setRGB(float r, float g, float b){

            m_rgb[0] = r;
            m_rgb[1] = g;
            m_rgb[2] = b;

        }

        void setThickness(float thickness){
            m_thickness = thickness;
        }

        float getThickness(){
            return m_thickness;
        }

        void addNode(float* new_node){

            if(!selfCheck()){
                return;
            }

            for(int i=0;i<m_node_dimension;++i){
                m_nodes.push_back(new_node[i]);
            }
            ++m_n_nodes;

        }

        void addNodes(float* new_nodes, int n_new_nodes){

            if(!selfCheck()){
                return;
            }

            if(n_new_nodes % m_node_dimension != 0){
                INFO_STREAM("You are trying to add half-nodes");
                return;
            }

            for(int i=0;i<n_new_nodes*m_node_dimension;++i){
                m_nodes.push_back(new_nodes[i]);
            }
            m_n_nodes+=n_new_nodes;

        }

        void setFloatsPerNode(int floats_per_node){
            m_node_dimension = floats_per_node;
        }

        int getNumberOfNodes(){
            return m_n_nodes;
        }

        void flipNodes(){

            if(m_n_nodes>0){
                for(int i=1;i<3*m_n_nodes;i+=3){
                    m_nodes[i] *= -1;
                }
            }
        }

        /* publish a line strip marker to visualize the main stem in rviz */
        void showInRviz(ros::Publisher* p_vis_pub){

            if(!selfCheck()){
                return;
            }

            int i;

            /* construct line strip marker object */
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/amigo/base_link";
            marker.header.stamp = ros::Time();
            marker.id = 0;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = m_thickness;
            marker.color.a = 1.0;
            marker.color.r = m_rgb[0];
            marker.color.g = m_rgb[1];
            marker.color.b = m_rgb[2];

            /* construct nodes point */
            for(i=0;i<m_n_nodes;++i){
                geometry_msgs::Point p;
                p.x = m_nodes[3*i];
                p.y = m_nodes[3*i+1];
                p.z = m_nodes[3*i+2];
                marker.points.push_back(p);
            }

            /* publish marker */
            p_vis_pub->publish( marker );

        }

        void printAll(){

            INFO_STREAM("===============");
            INFO_STREAM("Stem id: " << m_stem_id);
            INFO_STREAM("RGB: " << m_rgb[0] << " " << m_rgb[1] << " " << m_rgb[2] << " ");
            INFO_STREAM("Thickness: " << m_thickness << " cm");
            INFO_STREAM("Number of nodes: " << m_n_nodes);
            INFO_STREAM("Node dimension: " << m_node_dimension);

            std::stringstream nodes_stream;
            nodes_stream << "Nodes:" << std::endl;
            for(int i=0;i<m_n_nodes;++i){
                nodes_stream << "\t\t\t\t\t";
                for(int j=0;j<m_node_dimension;++j){
                    nodes_stream << m_nodes[i*m_node_dimension+j] << "\t";
                }
                nodes_stream << std::endl;
            }
            INFO_STREAM(nodes_stream.str());

            INFO_STREAM("===============");

        }

        ~VirtualStem(){
            // destructor
        }

};

class RobotConfig
{

    /* This class depends on
        <iostream>
        <kdl/tree.hpp>
        <kdl_parser/kdl_parser.hpp>
        <sensor_msgs/JointState.h>
        <ros/ros.h>
    */

    private:

        std::string m_name;             // name string of robot instance
        std::string m_urdf;             // xml formatted text containing urdf model
        bool m_preferred_arm_set;
        bool m_prefer_left_arm;         // preferring left arm if true, right arm if false
        KDL::Tree m_kinematic_tree;
        sensor_msgs::JointState m_arm_joint_msg;

    public:

        RobotConfig(const std::string name = "defaultRobot"){
               m_preferred_arm_set = false;
               m_name = name;
        }

        bool selfCheck(){

            if(!m_preferred_arm_set){
                INFO_STREAM("In RobotConfig of " << m_name << " preferred arm was not set");
                return false;
            }

            return true;
        }

        /* get robot description from ros parameter server */
        void loadUrdfFromRosparam(ros::NodeHandle n, const std::string urdf_rosparam){

            if (!n.getParam(urdf_rosparam, m_urdf)) {
                INFO_STREAM("Loading of robot urdf from rosparam \"" << urdf_rosparam << "\" failed!");
            }
            else{
                INFO_STREAM("Urdf loaded from rosparam \"" << urdf_rosparam << "\"");
            }
        }

        std::string getUrdf(){
            return m_urdf;
        }

        void loadKinematicTreeFromUrdf(){

            if (!kdl_parser::treeFromString(m_urdf, m_kinematic_tree)) {
                INFO_STREAM("Turning urdf into kdl tree failed!");
            }
            else{
                INFO_STREAM("Urdf has been turned in kdl tree.");
            }
        }

        KDL::Tree getKinematicTree(){
            return m_kinematic_tree;
        }


        void setLeftArmIsPreferred(){
            m_prefer_left_arm = true;

            m_arm_joint_msg.name.clear();
            m_arm_joint_msg.name.push_back("shoulder_roll_joint_left");
            m_arm_joint_msg.name.push_back("shoulder_pitch_joint_left");
            m_arm_joint_msg.name.push_back("shoulder_yaw_joint_left");
            m_arm_joint_msg.name.push_back("elbow_roll_joint_left");
            m_arm_joint_msg.name.push_back("elbow_pitch_joint_left");
            m_arm_joint_msg.name.push_back("wrist_pitch_joint_left");
            m_arm_joint_msg.name.push_back("wrist_yaw_joint_left");

            m_preferred_arm_set = true;
        }

        void setRightArmIsPreferred(){
            m_prefer_left_arm = false;

            m_arm_joint_msg.name.clear();
            m_arm_joint_msg.name.push_back("shoulder_roll_joint_right");
            m_arm_joint_msg.name.push_back("shoulder_pitch_joint_right");
            m_arm_joint_msg.name.push_back("shoulder_yaw_joint_right");
            m_arm_joint_msg.name.push_back("elbow_roll_joint_right");
            m_arm_joint_msg.name.push_back("elbow_pitch_joint_right");
            m_arm_joint_msg.name.push_back("wrist_pitch_joint_right");
            m_arm_joint_msg.name.push_back("wrist_yaw_joint_right");

            m_preferred_arm_set = true;
        }

        bool isLeftArmPreferred(){
            return m_prefer_left_arm;
        }

        bool isRightArmPreferred(){
            return !m_prefer_left_arm;
        }

        void publishInitialPose( ros::Publisher* p_arm_pub ){

            m_arm_joint_msg.header.stamp = ros::Time::now();
            m_arm_joint_msg.position.clear();

            /* amigo 'give' position */
            m_arm_joint_msg.position.push_back(0.0);
            m_arm_joint_msg.position.push_back(0.4);
            m_arm_joint_msg.position.push_back(-0.1);
            m_arm_joint_msg.position.push_back(0.0);
            m_arm_joint_msg.position.push_back(1.2);
            m_arm_joint_msg.position.push_back(0.0);
            m_arm_joint_msg.position.push_back(0.0);

            p_arm_pub->publish(m_arm_joint_msg);

        }

        void printAll(){

            INFO_STREAM("===============");
            INFO_STREAM("Robot name: " << m_name);

            if(m_urdf.empty())
                INFO_STREAM("URDF empty");
            else
                INFO_STREAM("URDF set, " << m_urdf.length() << " chars");

            INFO_STREAM("Preferred arm set: " << m_preferred_arm_set);
            INFO_STREAM("Preferring left arm: " << m_prefer_left_arm);

            INFO_STREAM("KDL tree:");
            INFO_STREAM("\tNumber of Joints: " << m_kinematic_tree.getNrOfJoints() );
            INFO_STREAM("\tNumber of Segments: " << m_kinematic_tree.getNrOfSegments() );
            INFO_STREAM("===============");

        }

        ~RobotConfig(){
            // destructor
        }
};



#endif // STEM_TRACKER_H
