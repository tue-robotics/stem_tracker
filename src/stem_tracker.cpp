
#include "stem_tracker.h"


/* config */

#define USE_LEFTARM false    // use left arm if true, else use right arm

double stemNodesXYZ[] = {0.3, 0.3, 0.4,     // nodes of a virtual stem,
                         0.35, 0.35, 0.6,   // list of coordinates xyzxyzxyz...
                         0.3, 0.35, 0.9,    // defined in amigo base_link
                         0.35, 0.4, 1.2,
                         0.3, 0.55, 1.4,
                         0.25, 0.6, 1.6};
\

int main(int argc, char** argv){

    int state = 0;
    int up = 1;

    /* initialize node */
    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    ros::Rate r(2); // hz

    ros::Publisher arm_pub;
    if (USE_LEFTARM)
        arm_pub = n.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 0);
    else
        arm_pub = n.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 0);

    sensor_msgs::JointState arm_joint_msg;

    /* initialize profiling */
    sp.initialize();

    /* update loop */
    while(ros::ok()){

        /* start sample timing, for profiling */
        sp.startTimer("main");

        /* publish linestrip marker to visualize stem */
        visualizeStem(vis_pub, stemNodesXYZ, sizeof(stemNodesXYZ)/sizeof(*stemNodesXYZ)/3);

        /* bring arm to initial position */
        arm_joint_msg = getInitialPosition(USE_LEFTARM);
        arm_pub.publish(arm_joint_msg);

        /* check have we reached end of stem */
        state += up;
        if(state>=(int)sizeof(stemNodesXYZ)/sizeof(double_t)/3-1 || state < 0){
            up = -up;
            ROS_INFO("reached end of stem");
            if(state<0){
                state = 0;
            }
        }
        else{
            ROS_INFO("subgoal accomplished");
        }

        /* stop and publish sample timing, for profiling */
        sp.stopTimer("main");
        sp.publish();

        /* selftrigger */
        r.sleep();
        ros::spinOnce();

    }

    return 0;
};


