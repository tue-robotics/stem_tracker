#ifndef ROBOTINTERFACE_H
#define ROBOTINTERFACE_H

#define INFO_STREAM     ROS_INFO_STREAM

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <kdl/jntarray.hpp>

#include "robotrepresentation.h"
#include "robotstatus.h"

class RobotInterface
{
private:
    RobotRepresentation* m_p_robot_representation;
    RobotStatus* m_p_robot_status;
    ros::NodeHandle m_node;
    ros::Publisher m_arm_ref_pub;
    ros::Publisher m_torso_ref_pub;
    ros::Subscriber m_arm_meas_sub;
    ros::Subscriber m_torso_meas_sub;
    ros::Subscriber m_whisker_pub_18, m_whisker_pub_17, m_whisker_pub_16, m_whisker_pub_15, m_whisker_pub_14, m_whisker_pub_13, m_whisker_pub_12, m_whisker_pub_11;
public:
    RobotInterface(ros::NodeHandle node, RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status);
    void connectToAmigoArm(const bool leftArmIsPreferred);
    void connectToAmigoTorso();
    void connectToWhiskers();

    void receivedAmigoTorsoMsg(const sensor_msgs::JointState& msg);
    void receivedAmigoArmMsg(const sensor_msgs::JointState& msg);

    void receivedWhisker18Msg(const std_msgs::Float32 & msg);
    void receivedWhisker17Msg(const std_msgs::Float32 & msg);
    void receivedWhisker16Msg(const std_msgs::Float32 & msg);
    void receivedWhisker15Msg(const std_msgs::Float32 & msg);
    void receivedWhisker14Msg(const std_msgs::Float32 & msg);
    void receivedWhisker13Msg(const std_msgs::Float32 & msg);
    void receivedWhisker12Msg(const std_msgs::Float32 & msg);
    void receivedWhisker11Msg(const std_msgs::Float32 & msg);

    void publishAmigoArmMessage(sensor_msgs::JointState arm_message);
    void publishJointPosRefs(KDL::JntArray joint_array);
    ~RobotInterface();
};

#endif // ROBOTINTERFACE_H
