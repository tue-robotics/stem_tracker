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
    */

    private:

        int m_stem_id;
        float m_rgb[3]; // between 0.0 - 1.0
        int m_n_nodes;
        int m_floats_per_node;
        std::vector<float> m_nodes;

    public:

        VirtualStem(int stem_id=-1){
            m_stem_id = stem_id;
            m_n_nodes = 0;
            m_floats_per_node = -1;
            m_rgb[0] = -1.0;
            m_rgb[1] = -1.0;
            m_rgb[2] = -1.0;
        }

        bool selfCheck(){

            bool IamOK = true;

            if(m_floats_per_node < 1){
                std::cout << "Illegal operation in stem with id " << m_stem_id << ", floats per node was not initialized or was set to value smaller than 1" << std::endl;
                IamOK = false;
            }

            for(int i=0;i<3;++i){
                if(m_rgb[i] < 0.0){
                    std::cout << "Illegal operation in stem with id " << m_stem_id << ", rgb value not initialized or set to value smaller than 0" << std::endl;
                    IamOK = false;
                }
            }

            return IamOK;
        }

        void setRGB(float r, float g, float b){

            m_rgb[0] = r;
            m_rgb[1] = g;
            m_rgb[2] = b;

        }

        void addNode(float* new_node){

            if(!selfCheck()){
                return;
            }

            for(int i=0;i<m_floats_per_node;++i){
                m_nodes.push_back(new_node[i]);
            }
            ++m_n_nodes;

        }

        void addNodes(float* new_nodes, int n_new_nodes){

            if(!selfCheck()){
                return;
            }

            if(n_new_nodes % m_floats_per_node != 0){
                std::cout << "You are trying to add half-nodes" << std::endl;
                return;
            }

            for(int i=0;i<n_new_nodes*m_floats_per_node;++i){
                m_nodes.push_back(new_nodes[i]);
            }
            m_n_nodes+=n_new_nodes;

        }

        void setFloatsPerNode(int floats_per_node){
            m_floats_per_node = floats_per_node;
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

        void printAll(){

            std::cout << "===============" << std::endl;

            std::cout << "Stem id: " << m_stem_id << std::endl;
            std::cout << "RGB: " << m_rgb[0] << " " << m_rgb[1] << " " << m_rgb[2] << " " << std::endl;
            std::cout << "Number of nodes: " << m_n_nodes << std::endl;
            std::cout << "Floats per node: " << m_floats_per_node << std::endl;
            std::cout << "Nodes: " << std::endl;
            for(int i=0;i<m_n_nodes;++i){
                std::cout << "\t";
                for(int j=0;j<m_floats_per_node;++j){
                    std::cout << m_nodes[i*m_floats_per_node+j] << "\t";
                }
                std::cout << std::endl;
            }
            std::cout << "===============" << std::endl;

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
        <ros/ros.h>
    */

    private:

        std::string m_name;             // name string of robot instance
        std::string m_urdf;             // xml formatted text containing urdf model
        bool m_prefer_left_arm;         // preferring left arm if true, right arm if false
        KDL::Tree m_kinematic_tree;

    public:

        RobotConfig(){
            m_name = "defaultRobot";
        }

        RobotConfig(const std::string name){
               m_name = name;
        }

        /* get robot description from ros parameter server */
        void loadUrdfFromRosparam(ros::NodeHandle n, const std::string urdf_rosparam){

            if (!n.getParam(urdf_rosparam, m_urdf)) {
                std::cout << "Loading of robot urdf from rosparam \"" << urdf_rosparam << "\" failed!" << std::endl;
            }
            else{
                std::cout << "Urdf loaded from rosparam \"" << urdf_rosparam << "\"" << std::endl;
            }
        }

        std::string getUrdf(){
            return m_urdf;
        }

        void loadKinematicTreeFromUrdf(){

            if (!kdl_parser::treeFromString(m_urdf, m_kinematic_tree)) {
                std::cout << "Turning urdf into kdl tree failed!" << std::endl;
            }
            else{
                std::cout << "Urdf has been turned in kdl tree." << std::endl;
            }
        }

        KDL::Tree getKinematicTree(){
            return m_kinematic_tree;
        }


        void setLeftArmIsPreferred(bool use_left_arm){
            m_prefer_left_arm = use_left_arm;
        }

        bool isLeftArmPreferred(){
            return m_prefer_left_arm;
        }

        void printAll(){

            std::cout << "===============" << std::endl;
            std::cout << "Robot name: " << m_name << std::endl;

            if(m_urdf.empty())
                std::cout << "URDF empty" << std::endl;
            else
                std::cout << "URDF set, " << m_urdf.length() << " chars" << std::endl;

            std::cout << "Preferring left arm: " << m_prefer_left_arm << std::endl;

            std::cout << "KDL tree:" << std::endl;
            std::cout << "\tNumber of Joints: " << m_kinematic_tree.getNrOfJoints() << std::endl;
            std::cout << "\tNumber of Segments: " << m_kinematic_tree.getNrOfSegments() << std::endl;
            std::cout << "===============" << std::endl;

        }

        ~RobotConfig(){
            // destructor
        }
};



sensor_msgs::JointState amigoGetInitialPosition( bool use_leftarm ){

    sensor_msgs::JointState arm_joint_msg;

    arm_joint_msg.header.stamp = ros::Time::now();

    arm_joint_msg.name.clear();
    arm_joint_msg.position.clear();

    if ( use_leftarm ){
        arm_joint_msg.name.push_back("shoulder_roll_joint_left");
        arm_joint_msg.name.push_back("shoulder_pitch_joint_left");
        arm_joint_msg.name.push_back("shoulder_yaw_joint_left");
        arm_joint_msg.name.push_back("elbow_roll_joint_left");
        arm_joint_msg.name.push_back("elbow_pitch_joint_left");
        arm_joint_msg.name.push_back("wrist_pitch_joint_left");
        arm_joint_msg.name.push_back("wrist_yaw_joint_left");
    }
    else {
        arm_joint_msg.name.push_back("shoulder_roll_joint_right");
        arm_joint_msg.name.push_back("shoulder_pitch_joint_right");
        arm_joint_msg.name.push_back("shoulder_yaw_joint_right");
        arm_joint_msg.name.push_back("elbow_roll_joint_right");
        arm_joint_msg.name.push_back("elbow_pitch_joint_right");
        arm_joint_msg.name.push_back("wrist_pitch_joint_right");
        arm_joint_msg.name.push_back("wrist_yaw_joint_right");
    }

    /* this corresponds to amigo 'give' position */
    arm_joint_msg.position.push_back(0.0);
    arm_joint_msg.position.push_back(0.4);
    arm_joint_msg.position.push_back(-0.1);
    arm_joint_msg.position.push_back(0.0);
    arm_joint_msg.position.push_back(1.2);
    arm_joint_msg.position.push_back(0.0);
    arm_joint_msg.position.push_back(0.0);

    return arm_joint_msg;
}


/* publish a line strip marker to visualize the
 * main stem in rviz */
void visualizeStem( ros::Publisher vis_pub, float *nodes, int n_nodes){

    int i;

    /* construct line strip marker object */
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/amigo/base_link";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03; // thickness in cm
    marker.color.a = 1.0;
    marker.color.r = 0.05;
    marker.color.g = 0.65;
    marker.color.b = 0.35;

    /* construct nodes point */
    for(i=0;i<n_nodes;++i){
        geometry_msgs::Point p;
        p.x = nodes[3*i];
        p.y = nodes[3*i+1];
        p.z = nodes[3*i+2];
        marker.points.push_back(p);
    }

    /* publish marker */
    vis_pub.publish( marker );

}


#endif // STEM_TRACKER_H
