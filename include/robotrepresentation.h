#ifndef ROBOTREPRESENTATION_H
#define ROBOTREPRESENTATION_H

#define INFO_STREAM     ROS_INFO_STREAM

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>


class RobotRepresentation
{
private:
    std::string m_name;             // name of robot instance
    urdf::Model m_urdf_model;       // urdf formatted model
    bool m_preferred_arm_set;
    bool m_prefer_left_arm;         // preferring left arm if true, right arm if false
    KDL::Tree m_kinematic_tree;
    KDL::Chain m_kinematic_chain;
    int m_n_joints_in_chain;
    KDL::JntArray m_q_min, m_q_max, m_q_seed;
    std::vector<std::string> m_q_joint_names;
    std::string m_root_link_name;
    std::string m_tip_link_name;

public:
    RobotRepresentation(const std::string name);
    bool selfCheck();
    KDL::JntArray getJointMinima();
    KDL::JntArray getJointMaxima();
    KDL::JntArray getJointSeeds();
    std::vector<std::string> getJointNames();
    void loadUrdfFromRosparam(ros::NodeHandle n, const std::string urdf_rosparam);
    urdf::Model getUrdfModel();
    void loadKinematicTreeFromUrdf();
    void loadKinematicChainFromTree(const std::string root_link_name, const std::string tip_link_name);
    void loadJointLimits();
    KDL::Tree getKinematicTree();
    KDL::Chain getKinematicChain();
    void setLeftArmIsPreferred();
    void setRightArmIsPreferred();
    bool isLeftArmPreferred();
    bool isRightArmPreferred();
    const std::string getName();
    sensor_msgs::JointState getAmigoInitialPoseMsg();
    void printAll();

    ~RobotRepresentation();
};

#endif // ROBOTREPRESENTATION_H
