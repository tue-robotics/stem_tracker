#ifndef WHISKERINTERPRETER_H
#define WHISKERINTERPRETER_H

#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>


#define INFO_STREAM     ROS_INFO_STREAM

class WhiskerInterpreter
{
    private:
        int m_n_whiskers;
        int m_gripper_id;
        float m_whisker_length;
        float m_gripper_diameter;
        float m_gripper_radius;
        int m_status; // 0 - unknown, 1 - gripper not around stem, 2 - gripper around stem but touching whiskers, 3 - gripper around stem and not touching whiskers
        std::vector<float> m_whisker_force;

    public:
        WhiskerInterpreter(int n_whiskers, int gripper_id, float whisker_length, float gripper_diameter);
        bool selfCheck();
        int getStatus();
        void simulateWhiskerGripper(std::vector<float> gripper_center, std::vector<float> stem_center);
        std::vector<float> getWhiskerForce();
        void showForceInRviz(ros::Publisher* p_vis_pub, std::vector<float> gripper_xyz);

        ~WhiskerInterpreter();
};

#endif // WHISKERINTERPRETER_H
