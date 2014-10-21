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
        std::vector<float> m_whisker_force;

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

        void simulateWhiskerGripper(std::vector<float> gripper_center, std::vector<float> stem_center){

            /* takes two doubles (xy) as coordinates of the gripper two doubles as coordinate of
             * the stem (both are in the same z-plane).
             * returns a force (xy) with origin at gripper center */

            m_whisker_force.clear();

            if(gripper_center.size() < 2 ){
                INFO_STREAM("in simulateWhiskerGripper gripper center xy needed!");
                return;
            }

            if(stem_center.size() < 2 ){
                INFO_STREAM("in simulateWhiskerGripper stem center xy needed!");
                return;
            }

            m_whisker_force.assign(2,0.0);

            double x_diff = gripper_center[0] - stem_center[0];
            double y_diff = gripper_center[1] - stem_center[1];

            if(sqrt( x_diff * x_diff + y_diff * y_diff ) > 10000.0 * m_gripper_diameter){
                INFO_STREAM("whiskers out of range!");
            }

            /* simulated force is 1-1 map from distance to force */
            m_whisker_force.at(0) = x_diff;
            m_whisker_force.at(1) = y_diff;

            INFO_STREAM("force_x = " << m_whisker_force[0] << " force_y " << m_whisker_force[1]);
        }

        std::vector<float> getWhiskerForce(){
            return m_whisker_force;
        }

        void showForceInRviz(ros::Publisher* p_vis_pub, std::vector<float> gripper_xyz){

            if(!selfCheck()){
                return;
            }

            if(gripper_xyz.size() < 3){
                INFO_STREAM("need gripper location xyz to visualize whisker force!");
                return;
            }

            /* construct line strip marker object */
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/amigo/base_link";
            marker.header.stamp = ros::Time();
            marker.id = 3;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.015;
            marker.color.a = 1.0f;
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;

            /* construct nodes point */
            geometry_msgs::Point p_start, p_end;

            p_start.x = gripper_xyz.at(0);
            p_start.y = gripper_xyz.at(1);
            p_start.z = gripper_xyz.at(2);
            marker.points.push_back(p_start);

            hier gebleven!!

            int sign;
            if(m_whisker_force[0] > 0 ){
                sign = 1;
            }else if (m_whisker_force[0] < 0){
                sign = -1;
            }else{
                sign = 0;
            }
            p_end.x = gripper_xyz.at(0) + m_whisker_force.at(0) - 0.05 * sign;

            if(m_whisker_force[1] > 0 ){
                sign = 1;
            }else if (m_whisker_force[1] < 0){
                sign = -1;
            }else{
                sign = 0;
            }
            p_end.y = gripper_xyz.at(1) + m_whisker_force.at(1) - 0.05 * sign;
            p_end.z = gripper_xyz.at(2);
            marker.points.push_back(p_end);

            /* publish marker */
            p_vis_pub->publish( marker );

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

        Assumption: stem derivative is always positive
    */

    private:

        int m_stem_id;
        float m_rgb[3]; // between 0.0 - 1.0

        int m_n_nodes;
        std::vector<float> m_x_nodes;
        std::vector<float> m_y_nodes;
        std::vector<float> m_z_nodes;

        float m_thickness; // in meters

    public:

        StemRepresentation(int stem_id=-1){
            m_stem_id = stem_id;
            m_n_nodes = 0;
            m_thickness = -1.0;
            m_rgb[0] = -1.0;
            m_rgb[1] = -1.0;
            m_rgb[2] = -1.0;
        }

        std::vector<float> getStemXYZatZ(float z){

            std::vector<float> xyz;

            if (m_z_nodes.back() < z){
                INFO_STREAM("z above stem length!");
                return xyz;
            }

            int index_first_above;
            for( index_first_above=0; index_first_above<m_z_nodes.size(); ++index_first_above){
                if(m_z_nodes.at(index_first_above) > z){
                    break;
                }
            }

            if (index_first_above < 1){
                INFO_STREAM("z below stem start or not enough nodes to calc xy for stem at z!");
                return xyz;
            }

            float atFraction;
            if(m_z_nodes.at(index_first_above) - m_z_nodes.at(index_first_above-1) <= 0.0){
                INFO_STREAM("stem bends down or goes horizontally, we don't support that. sorry");
                atFraction = 0.0;
            } else{
                atFraction = (z - m_z_nodes.at(index_first_above-1)) / (m_z_nodes.at(index_first_above) - m_z_nodes.at(index_first_above-1));
            }

            xyz.push_back( m_x_nodes.at(index_first_above-1) + atFraction * ( m_x_nodes.at(index_first_above) - m_x_nodes.at(index_first_above-1) ) );
            xyz.push_back( m_y_nodes.at(index_first_above-1) + atFraction * ( m_y_nodes.at(index_first_above) - m_y_nodes.at(index_first_above-1) ) );
            xyz.push_back(z);

            return xyz;

        }

        bool selfCheck(){

            bool IamOK = true;

            if( !( ( m_x_nodes.size() == m_y_nodes.size() ) && ( m_y_nodes.size() == m_z_nodes.size() ) ) ) {
                INFO_STREAM("in stem with id " << m_stem_id << ", vectors with node coordinates not of equal length!");
                INFO_STREAM("\t x_nodes.size() = " << m_x_nodes.size() << " y_nodes.size() = " << m_y_nodes.size() << " z_nodes.size() = " << m_z_nodes.size() << std::endl);
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

        void addNodesXYZ(std::vector<float> x, std::vector<float> y, std::vector<float> z){

            if(!selfCheck()){
                return;
            }

            for( int i=0; i<x.size(); ++i ){
                m_x_nodes.push_back(x.at(i));
                m_y_nodes.push_back(y.at(i));
                m_z_nodes.push_back(z.at(i));
            }

            m_n_nodes = m_x_nodes.size();

            selfCheck();

        }

        int getNumberOfNodes(){
            return m_n_nodes;
        }

        void flipNodes(){

            for(int i=0; i<m_y_nodes.size(); ++i){
                m_y_nodes.at(i) *= -1;
            }

        }

        /* publish a line strip marker to visualize the main stem in rviz */
        void showInRviz(ros::Publisher* p_vis_pub){

            if(!selfCheck()){
                return;
            }

            /* construct line strip marker object */
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/amigo/base_link";
            marker.header.stamp = ros::Time();
            marker.id = 0;
            marker.ns = "stem";
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = m_thickness;
            marker.color.a = 1.0;
            marker.color.r = m_rgb[0];
            marker.color.g = m_rgb[1];
            marker.color.b = m_rgb[2];

            /* construct nodes point */
            for(int i=0; i<m_x_nodes.size(); ++i){
                geometry_msgs::Point p;
                p.x = m_x_nodes.at(i);
                p.y = m_y_nodes.at(i);
                p.z = m_z_nodes.at(i);
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

            std::stringstream nodes_stream;
            nodes_stream << "Nodes:" << std::endl;
            for(int i=0;i<m_n_nodes;++i){
                nodes_stream << "\t\t\t\t\t" << m_x_nodes.at(i) << "\t" << m_y_nodes.at(i) << "\t" << m_z_nodes.at(i) << std::endl;
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

            int j=0;

            for(int i = 0; i < m_kinematic_chain.getNrOfSegments(); ++i){

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

        const std::string getName(){
            return m_name;
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

            std::stringstream tmp_stream;

            INFO_STREAM("===============");
            INFO_STREAM("Robot name: " << m_name);

            if (m_preferred_arm_set && m_prefer_left_arm){
                INFO_STREAM("Preferred arm is set to left arm.");
            } else if (m_preferred_arm_set && !m_prefer_left_arm){
                INFO_STREAM("Preferred arm is set to right arm.");
            } else {
                INFO_STREAM("Preferred arm was not set!");
            }

            INFO_STREAM("KDL tree:");
            INFO_STREAM("\tNumber of Joints: " << m_kinematic_tree.getNrOfJoints() );
            INFO_STREAM("\tNumber of Segments: " << m_kinematic_tree.getNrOfSegments() );
            INFO_STREAM("KDL chain:");
            INFO_STREAM("\tNumber of Joints: " << m_kinematic_chain.getNrOfJoints() );
            INFO_STREAM("\tNumber of Segments: " << m_kinematic_chain.getNrOfSegments() );

            tmp_stream.str(""); tmp_stream << "Jointnames:";
            for(int i=0;i<m_n_joints_in_chain;++i){
                tmp_stream << std::endl << "\t\t\t\t\t " << m_q_joint_names.at(i);
            }
            INFO_STREAM(tmp_stream.str());

            tmp_stream.str(""); tmp_stream << "Joint min:";
            for(int i=0;i<m_n_joints_in_chain;++i){
                tmp_stream << std::endl << "\t\t\t\t\t " << m_q_min.data[i];
            }
            INFO_STREAM(tmp_stream.str());

            tmp_stream.str(""); tmp_stream << "Joint max:";
            for(int i=0;i<m_n_joints_in_chain;++i){
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
     * stem_tracker RobotConfig
     * sensor_msgs/jointstate
     * kdl frame
     * <kdl/chainfksolverpos_recursive.hpp>
     */

    private:
        RobotConfig m_robot_config;
        KDL::JntArray m_joints_to_monitor; // order should be: torso / shoulder-jaw / shoulder-pitch / shoulder-roll / elbow-pitch / elbow-roll / wrist-pitch / wrist-yaw
        int m_n_joints_monitoring;

    public:

        RobotStatus(int n_joints_to_monitor, RobotConfig robot_config){

            m_robot_config = robot_config;

            if(n_joints_to_monitor <= 0){
                INFO_STREAM("trying to initialize robot status object with zero or negative number of joints to monitor!");
            }
            m_joints_to_monitor = KDL::JntArray(n_joints_to_monitor);
            m_n_joints_monitoring = n_joints_to_monitor;
        }

        bool selfCheck(){

            bool IamOK = true;

            if(m_n_joints_monitoring <= 0){
                INFO_STREAM("trying to use robot status object while number of joints to monitor is zero or negative!");
                IamOK = false;
            }

            return IamOK;
        }

        void receivedTorsoMsg(const sensor_msgs::JointState & msg){

            if(!selfCheck()){
                return;
            }

            m_joints_to_monitor(0) = msg.position[0];
        }

        void receivedArmMsg(const sensor_msgs::JointState & msg){

            if(!selfCheck()){
                return;
            }

            for(int i = 1; i < m_n_joints_monitoring; ++i){
                m_joints_to_monitor(i) = msg.position[i-1];
//                INFO_STREAM("received msg.position[" << i << "] = " << msg.position[i]);
            }
        }

        KDL::JntArray getJointStatus(){
            return m_joints_to_monitor;
        }

        std::vector<float> getGripperXYZ(){

            std::vector<float> gripper_xyz;

            KDL::ChainFkSolverPos_recursive forward_kinematics_solver = KDL::ChainFkSolverPos_recursive(m_robot_config.getKinematicChain());

            KDL::Frame cartpos;
            int fk_ret;

            fk_ret = forward_kinematics_solver.JntToCart(getJointStatus(),cartpos);

            gripper_xyz.push_back(cartpos.p.x());
            gripper_xyz.push_back(cartpos.p.y());
            gripper_xyz.push_back(cartpos.p.z());

            if( fk_ret < 0 ){
                INFO_STREAM("Warning: something went wrong in solving forward kinematics in getGripperXYZ");
            }

            return gripper_xyz;

        }

        void printAll(){

            INFO_STREAM("===============");
            INFO_STREAM("Robot status: ");
            INFO_STREAM("\t initialized with robot config of: " << m_robot_config.getName());
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
