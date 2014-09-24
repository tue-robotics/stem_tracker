#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H

/* ros includes */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

/* kdl includes */
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

/* amigo tooling includes */
#include <profiling/StatsPublisher.h>

/* for code profiling */
StatsPublisher sp;

/* for torso ref */
int count = 20;
int up = 1;

/* for setpoint generation */
double endEffDes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double prevEndEffDes[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/* general */
int ret, i; // return value

double determineTorsoReference(){

    if(count > 90 || count < 20)
        up = -up;

    count += up;

    return ( (double) count ) / 200.0;

}

void publishTorsoReference( ros::Publisher torsoRefPub, double ref ){

    sensor_msgs::JointState msg;

    msg.name.push_back("torso_joint");
    msg.position.push_back(ref);
    msg.header.stamp = ros::Time::now();

    torsoRefPub.publish(msg);
}

double euclDist(double* a, double* b){
    return sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2)+pow(a[2]-b[2],2));
}

void updateEndEffectorReference(double* endEffDes){

    for(i=0;i<3;++i){
        if ( euclDist(endEffDes,endXYZ) < 0.05 ){
            std::cout << "hoi" << std::endl;
            endEffDes[i] = endXYZ[i];
        }
        else
            endEffDes[i] += 0.1 * (endXYZ[i] - endEffDes[i]);
    }

    return;

}

void visualizeStem( ros::Publisher vis_pub){

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/amigo/base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "stem_ns";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = 2;
//    marker.pose.position.y = 1;
//    marker.pose.position.z = 1;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
//    marker.scale.y = 0.1;
//    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    geometry_msgs::Point p;
    p.x = startXYZ[0];//(int32_t)i - 50;
    p.y = startXYZ[1];
    p.z = startXYZ[2];

    marker.points.push_back(p);

    p.x = endXYZ[0];//(int32_t)i - 50;
    p.y = endXYZ[1];
    p.z = endXYZ[2];
    marker.points.push_back(p);

    vis_pub.publish( marker );

}

#endif // STEM_TRACKER_H
