#ifndef ROBOTSTATUS_H
#define ROBOTSTATUS_H

#define INFO_STREAM     ROS_INFO_STREAM

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>

#include "robotconfig.h"


class RobotStatus
{

    private:
        ros::Time m_last_update;
        KDL::JntArray m_joints_to_monitor; // order should be: torso / shoulder-jaw / shoulder-pitch / shoulder-roll / elbow-pitch / elbow-roll / wrist-pitch / wrist-yaw
        int m_n_joints_monitoring;
        std::vector<float> m_gripper_xyz;
        double m_up_to_date_threshold;
        KDL::Frame m_gripper_kdlframe;

    public:
        RobotStatus(int n_joints_to_monitor, RobotConfig* robot_config);

        bool selfCheck();
        void setUpToDateThreshold(double threshold);
        void receivedTorsoMsg(const sensor_msgs::JointState & msg);
        void receivedArmMsg(const sensor_msgs::JointState & msg);
        KDL::JntArray getJointStatus();
        std::vector<float> getGripperXYZ(RobotConfig* robot_config);
        KDL::Frame getGripperKDLframe(RobotConfig* robot_config);
        bool isGripperXYZvalid();
        ros::Time getLastUpdateTime();
        double getTimeSinceLastUpdate();
        bool isUpToDate();
        void printAll();

        ~RobotStatus();
};


#endif // ROBOTSTATUS_H
