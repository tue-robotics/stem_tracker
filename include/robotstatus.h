#ifndef ROBOTSTATUS_H
#define ROBOTSTATUS_H

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

class RobotRepresentation;

class RobotStatus
{
    private:
        ros::Time m_last_update;
        std::vector<ros::Time> m_last_update_times;
        std::vector<bool> m_starting_up;
        KDL::JntArray m_joints_to_monitor; // for amigo: torso / shoulder-jaw / shoulder-pitch / shoulder-roll / elbow-pitch / elbow-roll / wrist-pitch / wrist-yaw
        int m_n_joints_monitoring;
        std::vector<float> m_gripper_xyz;
        double m_up_to_date_threshold;
        KDL::Frame m_gripper_kdlframe;
        float m_pos_reached_threshold;
        RobotRepresentation* m_p_robot_representation;
        double m_xyz_reached_threshold;

    public:
        RobotStatus(RobotRepresentation* p_robot_representation);

        double setXYZreachedThreshold(double xyz_reached_threshold);
        void setPosReachedThreshold(float pos_reached_threshold);
        bool reachedPosition(KDL::JntArray reference);
        bool reachedPosition(std::vector<float> reference);
        void setUpToDateThreshold(double threshold);
        void updateJointStatus(KDL::JntArray updated_joint_status, std::vector<int> joints_updated);
        bool waitingForFirstStatusUpdate();

        bool isGripperXYZvalid();
        bool hasValidGripperXYZ();
        void updateGripperXYZ();

        const KDL::JntArray& getJointStatus() const { return m_joints_to_monitor; }
        const int& getNjointsMonitoring() const { return m_n_joints_monitoring; }
        const std::vector<float>& getGripperXYZ();
        const KDL::Frame& getGripperKDLframe();
        double getWorstCaseTimeSinceLastUpdate();

        bool isUpToDate();
        void printAll();

        ~RobotStatus();
};


#endif // ROBOTSTATUS_H
