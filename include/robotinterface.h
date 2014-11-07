#ifndef ROBOTINTERFACE_H
#define ROBOTINTERFACE_H

#define INFO_STREAM     ROS_INFO_STREAM

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <kdl/jntarray.hpp>

#include "robotrepresentation.h"
#include "robotstatus.h"

class RobotInterface
{
private:
    RobotRepresentation* m_p_robot_representation;
    RobotStatus* m_p_robot_status;
    ros::NodeHandle m_node;
    ros::Publisher m_vis_pub;           // visualization publisher
    ros::Publisher m_arm_ref_pub;       // arm reference publisher
    ros::Publisher m_torso_ref_pub;     // torso reference publisher
    ros::Subscriber m_arm_meas_sub;     // arm measurements subscriber
    ros::Subscriber m_torso_meas_sub;   // torso measurements subscriber
public:
    RobotInterface(ros::NodeHandle node, RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status);
    void receivedAmigoTorsoMsg(const sensor_msgs::JointState & msg);
    void receivedAmigoArmMsg(const sensor_msgs::JointState & msg);
    void publishAmigoArmMessage(sensor_msgs::JointState arm_message);
    void publishJointPosRefs(KDL::JntArray joint_array);
    ~RobotInterface();
};

#endif // ROBOTINTERFACE_H
