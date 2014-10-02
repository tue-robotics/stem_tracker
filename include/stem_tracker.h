#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H

/* ros includes */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>

/* amigo includes */
#include <profiling/StatsPublisher.h>

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


#endif // STEM_TRACKER_H
