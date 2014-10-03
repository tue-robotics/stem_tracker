#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H

/* ros includes */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>

/* amigo includes */
#include <profiling/StatsPublisher.h>

/* my/wbc includes */
#include "kinematic_tree.h"


/* ===== FUNCTIONS ================== */


Tree getKinematicTree(std::string xml_string){

    Tree kinematic_tree;

    if (!kdl_parser::treeFromString(xml_string, kinematic_tree.kdl_tree_)) {
        ROS_FATAL("Could not initialize kdl tree object");
    }
    ROS_INFO("KDL tree object initialized");

}

std::string getRobotDescription(ros::NodeHandle n, const std::string robot_description_rosparam){

    std::string robot_description_text;

    ROS_INFO("Reading robot description from rosparam: %s", robot_description_rosparam.c_str());

    if (!n.getParam(robot_description_rosparam, robot_description_text)) {
        ROS_FATAL("Could not load robot description!");
    }

    return robot_description_text;
}

sensor_msgs::JointState amigoGetInitialPosition( bool use_leftarm ){

    sensor_msgs::JointState arm_joint_msg;

    arm_joint_msg.header.stamp = ros::Time::now();

    arm_joint_msg.name.clear();
    arm_joint_msg.position.clear();

    if ( use_leftarm ){
        arm_joint_msg.name.push_back("shoulder_roll_joint_left");
        arm_joint_msg.name.push_back("shoulder_pitch_joint_left");
        arm_joint_msg.name.push_back("shoulder_yaw_joint_left");
        arm_joint_msg.name.push_back("elbow_roll_joint_left");
        arm_joint_msg.name.push_back("elbow_pitch_joint_left");
        arm_joint_msg.name.push_back("wrist_pitch_joint_left");
        arm_joint_msg.name.push_back("wrist_yaw_joint_left");
    }
    else {
        arm_joint_msg.name.push_back("shoulder_roll_joint_right");
        arm_joint_msg.name.push_back("shoulder_pitch_joint_right");
        arm_joint_msg.name.push_back("shoulder_yaw_joint_right");
        arm_joint_msg.name.push_back("elbow_roll_joint_right");
        arm_joint_msg.name.push_back("elbow_pitch_joint_right");
        arm_joint_msg.name.push_back("wrist_pitch_joint_right");
        arm_joint_msg.name.push_back("wrist_yaw_joint_right");
    }

    /* this corresponds to amigo 'give' position */
    arm_joint_msg.position.push_back(0.0);
    arm_joint_msg.position.push_back(0.4);
    arm_joint_msg.position.push_back(-0.1);
    arm_joint_msg.position.push_back(0.0);
    arm_joint_msg.position.push_back(1.2);
    arm_joint_msg.position.push_back(0.0);
    arm_joint_msg.position.push_back(0.0);

    return arm_joint_msg;
}


/* publish a line strip marker to visualize the
 * main stem in rviz */
void visualizeStem( ros::Publisher vis_pub, double *nodes, int n_nodes){

    int i;

    /* construct line strip marker object */
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/amigo/base_link";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03; // thickness in cm
    marker.color.a = 1.0;
    marker.color.r = 0.05;
    marker.color.g = 0.65;
    marker.color.b = 0.35;

    /* construct nodes point */
    for(i=0;i<n_nodes;++i){
        geometry_msgs::Point p;
        p.x = nodes[3*i];
        p.y = nodes[3*i+1];
        p.z = nodes[3*i+2];
        marker.points.push_back(p);
    }

    /* publish marker */
    vis_pub.publish( marker );

}


#endif // STEM_TRACKER_H
