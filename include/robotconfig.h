#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#define INFO_STREAM     ROS_INFO_STREAM

#include <iostream>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>


class RobotConfig
{
    friend class RobotStatus;

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
        std::string m_root_link_name;
        std::string m_tip_link_name;

    public:
        RobotConfig(const std::string name);

        bool selfCheck();
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
        sensor_msgs::JointState getInitialPoseMsg();
        void printAll();

        ~RobotConfig();
};

#endif // ROBOTCONFIG_H
