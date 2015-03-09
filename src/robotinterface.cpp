#include "robotinterface.h"
#include "loggingmacros.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "tue_msgs/GripperCommand.h"

void RobotInterface::connectToWhiskers()
{
    m_whisker_sub = m_node.subscribe("/amigo/whiskergripper/whisker_measurements", 1000, &RobotInterface::receivedWhiskerMsg, this);
}

void RobotInterface::connectToPressureSensors()
{
    m_pressure_sub = m_node.subscribe("/amigo/whiskergripper/top_measurements", 1000, &RobotInterface::receivedPressureSensorMsg, this);
}

void RobotInterface::receivedWhiskerMsg(const std_msgs::Float32MultiArray & msg)
{
    m_p_robot_status->updateWhiskerMeasurements(msg.data);
}

void RobotInterface::receivedPressureSensorMsg(const std_msgs::Float32MultiArray & msg)
{
    m_p_robot_status->updatePressureSensorMeasurements(msg.data);
}

void RobotInterface::connectToAmigoGripper(const bool leftArmIsPreferred)
{
    if(leftArmIsPreferred)
        m_gripper_ref_pub = m_node.advertise<tue_msgs::GripperCommand>("/amigo/left_arm/gripper/references",0);
    else
        m_gripper_ref_pub = m_node.advertise<tue_msgs::GripperCommand>("/amigo/right_arm/gripper/references",0);

    return;
}

void RobotInterface::connectToAmigoArm(const bool leftArmIsPreferred)
{
    if (leftArmIsPreferred)
        m_arm_ref_pub = m_node.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 0);
    else
        m_arm_ref_pub = m_node.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 0);

    if (leftArmIsPreferred)
        m_arm_meas_sub = m_node.subscribe("/amigo/left_arm/measurements", 1000, &RobotInterface::receivedAmigoArmMsg, this);
    else
        m_arm_meas_sub = m_node.subscribe("/amigo/right_arm/measurements", 1000, &RobotInterface::receivedAmigoArmMsg, this);
}

void RobotInterface::connectToAmigoTorso()
{
    m_torso_meas_sub = m_node.subscribe("/amigo/torso/measurements", 1000, &RobotInterface::receivedAmigoTorsoMsg, this);

    m_torso_ref_pub = m_node.advertise<sensor_msgs::JointState>("/amigo/torso/references", 0);
}

void RobotInterface::receivedAmigoTorsoMsg(const sensor_msgs::JointState & msg)
{
    KDL::JntArray joints_monitoring = m_p_robot_status->getJointStatus();
    std::vector<int> joints_updated;
    joints_updated.assign(joints_monitoring.rows(),0);

    for(uint i = 0; i < 1; ++i) // joint no. 1 in joints_monitoring is for amigo torso
    {
        joints_monitoring(i) = msg.position[i];
        joints_updated[i] = 1;
    }
    m_p_robot_status->updateJointStatus(joints_monitoring, joints_updated);
    return;
}

void RobotInterface::receivedAmigoArmMsg(const sensor_msgs::JointState & msg)
{
    KDL::JntArray joints_monitoring = m_p_robot_status->getJointStatus();
    std::vector<int> joints_updated;
    joints_updated.assign(joints_monitoring.rows(),0);

    for(uint i = 1; i < 8; ++i) // joints no. 2-8 in joints_monitoring are for amigo arm
    {
        joints_monitoring(i) = msg.position[i-1];
        joints_updated[i] = 1;
    }
    m_p_robot_status->updateJointStatus(joints_monitoring, joints_updated);
    return;
}

void RobotInterface::publishAmigoArmMessage(sensor_msgs::JointState arm_message)
{
    m_arm_ref_pub.publish(arm_message);
    return;
}

void RobotInterface::publishAmigoOpenGripperMessage()
{
    tue_msgs::GripperCommand msg;
    msg.direction = tue_msgs::GripperCommand::OPEN;
    msg.max_torque = 120;
    m_gripper_ref_pub.publish(msg);
    return;
}

void RobotInterface::publishAmigoJointPosRefs(KDL::JntArray q_out)
{
    if( q_out.rows() == m_p_robot_status->getJointStatus().rows())
    {
        std::vector<std::string> joint_names = m_p_robot_representation->getJointNames();

        sensor_msgs::JointState arm_ref;

        arm_ref.header.stamp = ros::Time::now();
        arm_ref.position.clear();

        for(int i = 1; i<8; ++i) // joints no. 2-8 in joints_monitoring are for amigo arm
        {
            arm_ref.position.push_back(q_out(i));
            arm_ref.name.push_back(joint_names[i]);
        }

        m_arm_ref_pub.publish(arm_ref);

        sensor_msgs::JointState torso_ref;

        torso_ref.header.stamp = ros::Time::now();
        torso_ref.position.clear();

        torso_ref.position.push_back(q_out(0));   // joint no. 1 in joints_monitoring is for amigo torso
        torso_ref.name.push_back(joint_names[0]);

        m_torso_ref_pub.publish(torso_ref);
    }
    else
        ERROR_STREAM("Publishing a number of joint position references not equal to the number of joints known in RobotStatus!");

    return;
}

RobotInterface::~RobotInterface()
{
    m_torso_meas_sub.shutdown();
    m_torso_ref_pub.shutdown();
    m_arm_meas_sub.shutdown();
    m_arm_ref_pub.shutdown();
    m_whisker_sub.shutdown();
    m_gripper_ref_pub.shutdown();
}
