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

#include "robotrepresentation.h"


class RobotStatus
{
    private:
        ros::Time m_last_update;
        KDL::JntArray m_joints_to_monitor; // order should be: torso / shoulder-jaw / shoulder-pitch / shoulder-roll / elbow-pitch / elbow-roll / wrist-pitch / wrist-yaw
        int m_n_joints_monitoring;
        std::vector<float> m_gripper_xyz;
        double m_up_to_date_threshold;
        KDL::Frame m_gripper_kdlframe;
        RobotRepresentation* m_p_robot_representation;

    public:
        RobotStatus(RobotRepresentation* p_robot_representation);
        bool selfCheck();
        void setUpToDateThreshold(double threshold);
        void updateJointStatus(KDL::JntArray updated_joint_status);
        KDL::JntArray getJointStatus();
        std::vector<float> getGripperXYZ();
        KDL::Frame getGripperKDLframe();
        bool isGripperXYZvalid();
        bool hasValidGripperXYZ();
        void updateGripperXYZ();
        ros::Time getLastUpdateTime();
        double getTimeSinceLastUpdate();
        bool isUpToDate();
        void printAll();

        ~RobotStatus();
};


#endif // ROBOTSTATUS_H
