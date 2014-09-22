#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/amigo/torso/references", 1000);
    ros::Rate loop_rate(10);

    int count = 20;
    int up = 1;
    double ref;

    while(ros::ok()){

        ROS_INFO("spinnin, ref = %f",ref);

        if(count > 95 || count < 20)
            up = -up;

        count=count+up;
        ref = ( (double)count ) /200.0;

        sensor_msgs::JointState msg;

        msg.name.push_back("torso_joint");
        msg.position.push_back(ref);
        msg.header.stamp = ros::Time::now();

        joint_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
};
