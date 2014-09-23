/* ros includes */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

/* kdl for ros includes */
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

/* amigo tooling includes */
#include <profiling/StatsPublisher.h>

/* header for this node */
#include "stem_tracker.h"

int main(int argc, char** argv){

    /* initialize node */
    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/amigo/torso/references", 1000);
    ros::Rate loop_rate(10);

    /* initialize profiling */
    sp.initialize();

    /* update loop */
    while(ros::ok()){

        /* start sample timing, for profiling */
        sp.startTimer("main");

        /* calculated desired joint reference */
        if(count > 90 || count < 20)
            up = -up;

        count=count+up;
        ref = ( (double)count ) / 200.0;

        /* publish updated joint reference */
        sensor_msgs::JointState msg;

        msg.name.push_back("torso_joint");
        msg.position.push_back(ref);
        msg.header.stamp = ros::Time::now();

        joint_pub.publish(msg);
        ROS_INFO("spinnin, ref = %f",ref);

        /* stop and publish sample time, for profiling */
        sp.stopTimer("main");
        sp.publish();

        /* wait until next sample */
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
};
