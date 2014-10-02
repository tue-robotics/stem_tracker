
/* config */

int rate = 100;             // sample rate of this node
#define USE_LEFTARM true    // use left arm if true, else use right arm

double stemNodesXYZ[] = {0.3, 0.3, 0.4,
                         0.35, 0.35, 0.6,
                         0.3, 0.35, 0.9,
                         0.35, 0.4, 1.2,
                         0.3, 0.55, 1.4,
                         0.25, 0.6, 1.6};
\

#include "stem_tracker.h"


int main(int argc, char** argv){

    /* initialize node */
    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    if (USE_LEFTARM)
        ros::Publisher arm_pub = n.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 0);
    else
        ros::Publisher arm_pub = n.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 0);

    sensor_msgs::JointState joint_msg;

    /* initialize profiling */
    sp.initialize();

    /* update loop */
    while(ros::ok()){

        /* start sample timing, for profiling */
        sp.startTimer("main");

        /* publish line strip marker to visualize stem */
        visualizeStem(vis_pub);


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
        ros::spinOnce();

    }

    return 0;
};


