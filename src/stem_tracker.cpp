#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>

/* config */
int rate = 1;                               // sample rate of this node
double startXYZ[3] = {0.45, 0.3, 0.3};      // starting coordinate stem, in amigo base_link
double endXYZ[3] = {0.25, 0.5, 1.6};        // end coordinate stem
std::string wbcCall;                        // string to call whole body controller (wbc)
#define WBC_CALL_PREFIX "rosrun amigo_whole_body_controller TestCartesianMotionObjective.py "
#define TOL_ENDPOINT 0.05                   // subtarget becomes end-point if end-effector approached by less than this distance

#include "stem_tracker.h"

int main(int argc, char** argv){

    /* initialize node */
    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    ros::Rate loop_rate( rate );

    /* initialize profiling */
    sp.initialize();

    /* initialize subtarget at start position */
    memcpy(endEffDes,startXYZ,sizeof(startXYZ));

    /* update loop */
    while(ros::ok()){

        /* start sample timing, for profiling */
        sp.startTimer("main");

        /* publish line strip marker to visualize stem */
        visualizeStem(vis_pub);

        /* construct motion primitive call to wbc */
        wbcCall.clear();
        wbcCall.append(WBC_CALL_PREFIX);
        for(i=0;i<6;++i){
            std::ostringstream strs;
            strs << endEffDes[i];
            wbcCall.append(strs.str());
            wbcCall.append(" ");
        }

        wbcCall.append(">> /dev/null");

        /* add motion primitive to whole body controller */
        ret = system(wbcCall.c_str());

        /* if subtarget reached, update */
        updateEndEffRef(endEffDes);

        /* stop and publish sample timing, for profiling */
        sp.stopTimer("main");
        sp.publish();

        /* wait until next sample */
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
};


