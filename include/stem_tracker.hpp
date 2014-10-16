#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H

/* c++ includes */
#include <vector>
#include <map>
#include <iostream>
#include <cmath>

#include <boost/shared_ptr.hpp>

/* ros includes */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

/* kdl includes */
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

//#include <kdl/treejnttojacsolver.hpp>

/* amigo includes */
#include <profiling/StatsPublisher.h>


class WhiskerInterpreter
{
    /* This class depends on:
     *
     * <vector>
     * <cmath>
     */

    private:
        int m_n_whiskers;
        int m_gripper_id;
        float m_whisker_length;
        float m_gripper_diameter;

    public:

        WhiskerInterpreter(int n_whiskers, int gripper_id, float whisker_length, float gripper_diameter){
            m_n_whiskers = n_whiskers;
            m_gripper_id = gripper_id;
            m_whisker_length = whisker_length;
            m_gripper_diameter = gripper_diameter;
        }

        double selfCheck(){

            bool IamOK = true;

            if (m_n_whiskers <= 0){
                INFO_STREAM("In whisker gripper with id " << m_gripper_id << " number of whiskers set to zero or negative number");
                IamOK = false;
            }

            return IamOK;
        }

        std::vector<double> simulateWhiskerGripper(std::vector<double> gripper_center, std::vector<double> stem_center){

            /* takes two doubles (xy) as coordinates of the gripper two doubles as coordinate of
             * the stem (both are in the same z-plane).
             * returns a force (xy) with origin at gripper center */

            if(gripper_center.size() != 2 ){
                INFO_STREAM("in simulateWhiskerGripper the gripper center should be of dimension 2!");
            }

            if(stem_center.size() != 2 ){
                INFO_STREAM("in simulateWhiskerGripper the stem center should be of dimension 2!");
            }

            std::vector<double> force;
            force.assign(2,0.0);

            double x_diff = gripper_center[0] - stem_center[0];
            double y_diff = gripper_center[1] - stem_center[1];

            if(sqrt( x_diff * x_diff + y_diff * y_diff ) > 0.5 * m_gripper_diameter){
                INFO_STREAM("whiskers out of range!");
                return force;
            }

            /* simulated force is 1-1 map from distance to force */
            force[0] = x_diff;
            force[1] = y_diff;
            return force;
        }

        ~WhiskerInterpreter(){
            //destructor
        }
};

class StemRepresentation
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
        float m_thickness; // in meters
        std::vector<float> m_nodes;

    public:

        StemRepresentation(int stem_id=-1){
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

        void setNodeDimension(int node_dimension){
            m_node_dimension = node_dimension;
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
            INFO_STREAM("Thickness: " << m_thickness << " meters");
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

        ~StemRepresentation(){
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
        <kdl/chain.hpp>
        <kdl/jntarray.hpp>
        <urdf/model.h>
    */

    private:

        std::string m_name;             // name of robot instance
        urdf::Model m_urdf_model;       // urdf formatted model
        bool m_preferred_arm_set;
        bool m_prefer_left_arm;         // preferring left arm if true, right arm if false
        KDL::Tree m_kinematic_tree;
        sensor_msgs::JointState m_arm_joint_msg;
        KDL::Chain m_kinematic_chain;
        int m_n_joints_in_chain;
        KDL::JntArray m_q_min, m_q_max;
        std::vector<std::string> m_q_joint_names;

    public:

        RobotConfig(const std::string name = "defaultRobot"){
               m_preferred_arm_set = false;
               m_n_joints_in_chain = -1;
               m_name = name;
        }

        bool selfCheck(){

            if(!m_preferred_arm_set){
                INFO_STREAM("In RobotConfig of " << m_name << " preferred arm was not set");
                return false;
            }

            if(m_n_joints_in_chain <= 0){
                INFO_STREAM("In RobotConfig of " << m_name << " kinematic chain contains no joints");
            }

            return true;
        }

        /* get robot description from ros parameter server */
        void loadUrdfFromRosparam(ros::NodeHandle n, const std::string urdf_rosparam){

            std::string urdf;
            if (!n.getParam(urdf_rosparam, urdf)) {
                INFO_STREAM("Loading of robot urdf from rosparam \"" << urdf_rosparam << "\" failed!");
                return;
            }
            else{
                INFO_STREAM("Urdf xml loaded from rosparam \"" << urdf_rosparam << "\"");
            }

            if (!m_urdf_model.initString(urdf))
            {
                INFO_STREAM("Could not initialize urdf model from xml");
            }
        }

        urdf::Model getUrdfModel(){
            return m_urdf_model;
        }

        void loadKinematicTreeFromUrdf(){

            if (!kdl_parser::treeFromUrdfModel(m_urdf_model, m_kinematic_tree)) {
                INFO_STREAM("Turning urdf model into kdl tree failed!");
            }
            else{
                INFO_STREAM("Urdf model has been turned in kdl tree.");
            }
        }

        void loadKinematicChainFromTree(const std::string root_link_name, const std::string tip_link_name){

            if (!m_kinematic_tree.getChain(root_link_name, tip_link_name, m_kinematic_chain)){
                INFO_STREAM("Could not initialize chain object");
            }
            INFO_STREAM("Kinematic chain initialized from tree");

            m_n_joints_in_chain = m_kinematic_chain.getNrOfJoints();
        }

        void loadJointLimits(){

            m_q_min.resize(m_n_joints_in_chain);
            m_q_max.resize(m_n_joints_in_chain);
            m_q_joint_names.resize(m_n_joints_in_chain);

            int i,j=0;

            for(i = 0; i < m_kinematic_chain.getNrOfSegments(); ++i){

                const KDL::Joint& kdl_joint = m_kinematic_chain.getSegment(i).getJoint();

                if (kdl_joint.getType() != KDL::Joint::None){

                    m_q_joint_names[j] = kdl_joint.getName();

                    boost::shared_ptr<const urdf::Joint> urdf_joint = m_urdf_model.getJoint(kdl_joint.getName());

                    urdf_joint = m_urdf_model.getJoint(kdl_joint.getName());

                    if (urdf_joint && urdf_joint->limits){
                        m_q_min(j) = urdf_joint->limits->lower;
                        m_q_max(j) = urdf_joint->limits->upper;
                    } else{
                        m_q_min(j) = -1e9;
                        m_q_max(j) = 1e9;
                    }
                    ++j;
                }
            }

        }

        KDL::Tree getKinematicTree(){
            return m_kinematic_tree;
        }

        KDL::Chain getKinematicChain(){
            return m_kinematic_chain;
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

        sensor_msgs::JointState getInitialPoseMsg(){

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

            return m_arm_joint_msg;

        }

        void printAll(){

            int i;
            std::stringstream tmp_stream;

            INFO_STREAM("===============");
            INFO_STREAM("Robot name: " << m_name);

            INFO_STREAM("Preferred arm is set: " << m_preferred_arm_set);
            INFO_STREAM("Preferring left arm: " << m_prefer_left_arm);

            INFO_STREAM("KDL tree:");
            INFO_STREAM("\tNumber of Joints: " << m_kinematic_tree.getNrOfJoints() );
            INFO_STREAM("\tNumber of Segments: " << m_kinematic_tree.getNrOfSegments() );
            INFO_STREAM("KDL chain:");
            INFO_STREAM("\tNumber of Joints: " << m_kinematic_chain.getNrOfJoints() );
            INFO_STREAM("\tNumber of Segments: " << m_kinematic_chain.getNrOfSegments() );

            tmp_stream.str(""); tmp_stream << "Jointnames:";
            for(i=0;i<m_n_joints_in_chain;++i){
                tmp_stream << std::endl << "\t\t\t\t\t " << m_q_joint_names.at(i);
            }
            INFO_STREAM(tmp_stream.str());

            tmp_stream.str(""); tmp_stream << "Joint min:";
            for(i=0;i<m_n_joints_in_chain;++i){
                tmp_stream << std::endl << "\t\t\t\t\t " << m_q_min.data[i];
            }
            INFO_STREAM(tmp_stream.str());

            tmp_stream.str(""); tmp_stream << "Joint max:";
            for(i=0;i<m_n_joints_in_chain;++i){
                tmp_stream << std::endl << "\t\t\t\t\t " << m_q_max.data[i];
            }
            INFO_STREAM(tmp_stream.str());

            INFO_STREAM("===============");

        }

        ~RobotConfig(){
            // destructor
        }
};

class RobotStatus
{
    /* This class depends on:
     *
     */

    private:
        KDL::JntArray m_joints_to_monitor; // order should be: torso / shoulder-jaw / shoulder-pitch / shoulder-roll / elbow-pitch / elbow-roll / wrist-pitch / wrist-yaw
        int m_n_joints_monitoring;

    public:

        RobotStatus(int n_joints_to_monitor){
            m_joints_to_monitor = KDL::JntArray(n_joints_to_monitor);
            m_n_joints_monitoring = n_joints_to_monitor;
        }

        void receivedTorsoMsg(const sensor_msgs::JointState & msg){
            m_joints_to_monitor(0) = msg.position[0];
        }

        void receivedArmMsg(const sensor_msgs::JointState & msg){
            for(int i = 1; i < m_n_joints_monitoring; ++i){
                m_joints_to_monitor(i) = msg.position[i];
//                INFO_STREAM("received msg.position[" << i << "] = " << msg.position[i]);
            }
        }

        KDL::JntArray getJointStatus(){
            return m_joints_to_monitor;
        }

        void printAll(){

            INFO_STREAM("===============");
            INFO_STREAM("Robot status: ");
            for(int i; i<m_n_joints_monitoring; ++i){
                INFO_STREAM("\t" << m_joints_to_monitor(i));
            }
            INFO_STREAM("===============");
        }

        ~RobotStatus(){
            // destructor
        }
};

#endif // STEM_TRACKER_H
