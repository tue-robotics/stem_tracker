#ifndef ROBOTINTERFACE_H
#define ROBOTINTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <kdl/jntarray.hpp>

class RobotRepresentation;
class RobotStatus;

class RobotInterface
{
private:
    RobotRepresentation* m_p_robot_representation;
    RobotStatus* m_p_robot_status;
    ros::NodeHandle m_node;
    ros::Publisher m_arm_ref_pub;
    ros::Publisher m_torso_ref_pub;
    ros::Publisher m_gripper_ref_pub;
    ros::Subscriber m_arm_meas_sub;
    ros::Subscriber m_torso_meas_sub;
    ros::Subscriber m_whisker_sub;
    ros::Subscriber m_pressure_sub;
    bool m_debug_joint_max;
public:
    RobotInterface(ros::NodeHandle node, RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status)
        : m_node(node), m_p_robot_representation(p_robot_representation), m_p_robot_status(p_robot_status) {}

    void connectToAmigoArm(const bool leftArmIsPreferred);
    void connectToAmigoGripper(const bool leftArmIsPreferred);
    void connectToAmigoTorso();
    void connectToWhiskers();
    void connectToPressureSensors();

    void receivedAmigoTorsoMsg(const sensor_msgs::JointState& msg);
    void receivedAmigoArmMsg(const sensor_msgs::JointState& msg);
    void receivedWhiskerMsg(const std_msgs::Float32MultiArray& msg);
    void receivedPressureSensorMsg(const std_msgs::Float32MultiArray& msg);

    void publishAmigoArmMessage(sensor_msgs::JointState arm_message);
    void publishAmigoJointPosRefs(KDL::JntArray joint_array);
    void publishAmigoOpenGripperMessage();

    void checkForJointLimits(const float& q_val, const int& i) const;
    inline void setDebugJointMax(bool debug) { m_debug_joint_max = debug; }

    ~RobotInterface();
};

#endif // ROBOTINTERFACE_H
