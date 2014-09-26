#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>

/* config */

int rate = 100;             // sample rate of this node
#define WBC_STIFFNESS 6.0   // stiffness for motion objective send to whole body controller
#define TIMEOUT 10          // max seconds waiting until subtarget supposed to be reached

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
    actionlib::SimpleActionClient<amigo_whole_body_controller::ArmTaskAction> call_action("add_motion_objective",true);

    ROS_INFO("waiting for action server");
    call_action.waitForServer();
    ROS_INFO("found action server");

    /* initialize profiling */
    sp.initialize();

    /* update loop */
    while(ros::ok()){

        /* start sample timing, for profiling */
        sp.startTimer("main");

        /* publish line strip marker to visualize stem */
        visualizeStem(vis_pub);

        ROS_INFO("State = %d",state);
        /* construct and send motion objective to wbc */
        call_action.sendGoal( obtainStemGoal(state) );

        /* wait until goal is accomplished */
        call_action.waitForResult(ros::Duration(4.0));
        actionlib::SimpleClientGoalState goalState = call_action.getState();
        if (!goalState.isDone()){
            ROS_ERROR("wbc action server reports it is not done yet, I'll continue anyway");
        }

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


