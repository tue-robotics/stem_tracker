#include "robotinterface.h"

RobotInterface::RobotInterface(ros::NodeHandle node, RobotRepresentation* p_robot_representation, RobotStatus* p_robot_status)
{
    m_node = node;
    m_p_robot_representation = p_robot_representation;
    m_p_robot_status = p_robot_status;

    if (p_robot_representation->isLeftArmPreferred())
        m_arm_ref_pub = m_node.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 0);
    else if (p_robot_representation->isRightArmPreferred())
        m_arm_ref_pub = m_node.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 0);
    else
        INFO_STREAM("trying to initialize robot interface without robot config properly configured");

    m_torso_meas_sub = m_node.subscribe("/amigo/torso/measurements", 1000, &RobotInterface::receivedAmigoTorsoMsg, this);

    m_torso_ref_pub = m_node.advertise<sensor_msgs::JointState>("/amigo/torso/references", 0);

    if (p_robot_representation->isLeftArmPreferred())
        m_arm_meas_sub = m_node.subscribe("/amigo/left_arm/measurements", 1000, &RobotInterface::receivedAmigoArmMsg, this);
    else if (p_robot_representation->isRightArmPreferred())
        m_arm_meas_sub = m_node.subscribe("/amigo/right_arm/measurements", 1000, &RobotInterface::receivedAmigoArmMsg, this);
    else
        INFO_STREAM("trying to initialize robot interface without robot config properly configured");
}


void RobotInterface::receivedAmigoTorsoMsg(const sensor_msgs::JointState & msg)
{
    KDL::JntArray joints_monitoring = m_p_robot_status->getJointStatus();
    joints_monitoring(0) = msg.position[0];
    m_p_robot_status->updateJointStatus(joints_monitoring);
    return;
}

void RobotInterface::receivedAmigoArmMsg(const sensor_msgs::JointState & msg)
{
    KDL::JntArray joints_monitoring = m_p_robot_status->getJointStatus();
    for(int i = 1; i < 8; ++i)
        joints_monitoring(i) = msg.position[i-1];
    m_p_robot_status->updateJointStatus(joints_monitoring);
    return;
}

void RobotInterface::publishAmigoArmMessage(sensor_msgs::JointState arm_message)
{
    m_arm_ref_pub.publish(arm_message);
    return;
}

void RobotInterface::publishJointPosRefs(KDL::JntArray q_out)
{
    if( q_out.rows() == m_p_robot_status->getJointStatus().rows())
    {
        std::vector<std::string> joint_names = m_p_robot_representation->getJointNames();

        sensor_msgs::JointState arm_ref;

        arm_ref.header.stamp = ros::Time::now();
        arm_ref.position.clear();

        for(int i = 1; i<8; ++i)
        {
            arm_ref.position.push_back(q_out(i));
            arm_ref.name.push_back(joint_names[i]);
        }

        m_arm_ref_pub.publish(arm_ref);

        sensor_msgs::JointState torso_ref;

        torso_ref.header.stamp = ros::Time::now();
        torso_ref.position.clear();

        torso_ref.position.push_back(q_out(0));
        torso_ref.name.push_back(joint_names[0]);

        m_torso_ref_pub.publish(torso_ref);
    }
    else
        INFO_STREAM("trying to publish a number of joint position references not equal to the number of joints in robot status");

    return;
}

RobotInterface::~RobotInterface()
{
    m_torso_meas_sub.shutdown();
    m_torso_ref_pub.shutdown();
    m_arm_meas_sub.shutdown();
    m_arm_ref_pub.shutdown();
}
