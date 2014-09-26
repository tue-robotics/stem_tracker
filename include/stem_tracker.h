#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H

/* ros includes */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>

/* amigo includes */
#include <profiling/StatsPublisher.h>
#include <amigo_whole_body_controller/ArmTaskAction.h>

/* for code profiling */
StatsPublisher sp;

/* subtarget global data */
int state = 0;
int up = 1;

/* general */
int i;


/* =============================================== */



/* publish a line strip marker to visualize the
 * main stem in rviz */
void visualizeStem( ros::Publisher vis_pub){

    /* construct line strip marker object */
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/amigo/base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "stem_ns";
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
    for(i=0;i<(int)sizeof(stemNodesXYZ)/sizeof(double_t)/3;++i){
        geometry_msgs::Point p;
        p.x = stemNodesXYZ[3*i];
        p.y = stemNodesXYZ[3*i+1];
        p.z = stemNodesXYZ[3*i+2];
        marker.points.push_back(p);
    }

    /* publish marker */
    vis_pub.publish( marker );

}

amigo_whole_body_controller::ArmTaskGoal obtainStemGoal(int state){

    amigo_whole_body_controller::ArmTaskGoal stem_goal;

    stem_goal.goal_type = "grasp";
    stem_goal.position_constraint.header.frame_id = "base_link";
    stem_goal.position_constraint.link_name = "grippoint_left";
    stem_goal.position_constraint.target_point_offset.x = 0.0;
    stem_goal.position_constraint.target_point_offset.y = 0.0;
    stem_goal.position_constraint.target_point_offset.z = 0.0;
    stem_goal.position_constraint.position.x = stemNodesXYZ[3*state];
    stem_goal.position_constraint.position.y = stemNodesXYZ[3*state+1];
    stem_goal.position_constraint.position.z = stemNodesXYZ[3*state+2];
    stem_goal.orientation_constraint.header.frame_id = "base_link";
    stem_goal.orientation_constraint.link_name = "grippoint_left";
    stem_goal.orientation_constraint.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    stem_goal.stiffness.force.x =  70.0 * WBC_STIFFNESS;
    stem_goal.stiffness.force.y =  70.0 * WBC_STIFFNESS;
    stem_goal.stiffness.force.z =  50.0 * WBC_STIFFNESS;
    stem_goal.stiffness.torque.x = 5.0 * WBC_STIFFNESS;
    stem_goal.stiffness.torque.y = 5.0 * WBC_STIFFNESS;
    stem_goal.stiffness.torque.z = 5.0 * WBC_STIFFNESS;

    return stem_goal;
}

#endif // STEM_TRACKER_H
