#ifndef ROBOTSTATUS_H
#define ROBOTSTATUS_H

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
        float m_pos_reached_threshold;
        RobotRepresentation* m_p_robot_representation;
        double m_xyz_reached_threshold;

    public:
        RobotStatus(RobotRepresentation* p_robot_representation);
        bool selfCheck();
        double setXYZreachedThreshold(double xyz_reached_threshold);
        void setPosReachedThreshold(float pos_reached_threshold);
        bool reachedPosition(KDL::JntArray reference);
        bool reachedPosition(std::vector<float> reference);
        void setUpToDateThreshold(double threshold);
        void updateJointStatus(KDL::JntArray updated_joint_status);
        KDL::JntArray getJointStatus();
        std::vector<float> getGripperXYZ();
        KDL::Frame getGripperKDLframe();
        bool isGripperXYZvalid();
        bool hasValidGripperXYZ();
        void updateGripperXYZ();
        int getNjointsMonitoring();
        ros::Time getLastUpdateTime();
        double getTimeSinceLastUpdate();
        bool isUpToDate();
        void printAll();

        ~RobotStatus();
};


#endif // ROBOTSTATUS_H
