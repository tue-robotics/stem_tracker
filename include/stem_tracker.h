#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H

/* ros includes */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

/* amigo tooling includes */
#include <profiling/StatsPublisher.h>

/* for code profiling */
StatsPublisher sp;

/* subtarget global data */
double endEffDes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double prevEndEffDes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/* general */
int ret, i;


/* =============================================== */


/* calculate euclidian distance between two 3d vectors */
double euclDist(double* a, double* b){
    return sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2)+pow(a[2]-b[2],2));
}

/* check if sufficiently close to end-point, if so than set
 * subtarget to end-point, if than add ten percent of path to go
 * note: this will lead to zeno-behavior so endpoint tol needs to
 * be sufficiently large */
void updateEndEffRef(double* endEffDes){

    for(i=0;i<3;++i){
        if ( euclDist(endEffDes,endXYZ) < TOL_ENDPOINT )
            endEffDes[i] = endXYZ[i];
        else
            endEffDes[i] += 0.1 * (endXYZ[i] - endEffDes[i]);
    }

    return;

}

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

    /* construct starting point */
    geometry_msgs::Point p;
    p.x = startXYZ[0];
    p.y = startXYZ[1];
    p.z = startXYZ[2];
    marker.points.push_back(p);

    /* construct end point */
    p.x = endXYZ[0];
    p.y = endXYZ[1];
    p.z = endXYZ[2];
    marker.points.push_back(p);

    /* publish marker */
    vis_pub.publish( marker );

}

#endif // STEM_TRACKER_H
