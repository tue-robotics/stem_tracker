#include <string>
#include <stdlib.h>
#include <iostream>
#include <math.h>

/* config */
bool torsoUpAndDown = false;
bool useWholeBodyContr = true;
int rate = 1; // 10 for torsoUpAndDown, 1 for wholeBodyContr
double startXYZ[3] = {0.45, 0.3, 0.3};
double endXYZ[3] = {0.25, 0.5, 1.6};
std::string wbcCall;
#define WBC_CALL_PREFIX "rosrun amigo_whole_body_controller TestCartesianMotionObjective.py "


#include "stem_tracker.h"

int main(int argc, char** argv){

    /* initialize node */
    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;
    ros::Publisher torsoRefPub = n.advertise<sensor_msgs::JointState>("/amigo/torso/references", 1000);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    ros::Rate loop_rate( rate );

    /* initialize profiling */
    sp.initialize();

    if (useWholeBodyContr){
        memcpy(endEffDes,startXYZ,sizeof(startXYZ));
    }

    /* update loop */
    while(ros::ok()){

        /* start sample timing, for profiling */
        sp.startTimer("main");

        /* move torso up and down */
        if (torsoUpAndDown)
            publishTorsoReference( torsoRefPub, determineTorsoReference() );

        if (useWholeBodyContr){

            visualizeStem(vis_pub);

            wbcCall.clear();
            wbcCall.append(WBC_CALL_PREFIX);
            for(i=0;i<6;++i){
                std::ostringstream strs;
                strs << endEffDes[i];
                wbcCall.append(strs.str());
                wbcCall.append(" ");
            }

            wbcCall.append(">> /dev/null");
            std::cout << wbcCall << std::endl;
            ret = system(wbcCall.c_str());
            ROS_INFO("ret = %d",ret);
            updateEndEffectorReference(endEffDes);
        }

        /* stop and publish time, for profiling */
        sp.stopTimer("main");
        sp.publish();

        /* wait until next sample */
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
};


