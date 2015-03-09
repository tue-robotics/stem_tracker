/* c++ includes */
#include <iostream>

/* ros includes */
#include <ros/ros.h>
#include <ros/package.h>

/* amigo includes */
#include <profiling/StatsPublisher.h>

/* stem tracking includes */
#include "debugfunctions.h"
#include "whiskergripperinterpreter.h"
#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "visualizationinterface.h"
#include "robotinterface.h"
#include "stemtrackcontroller.h"
#include "stemtrackmonitor.h"
#include "stemtrackconfigurer.h"
#include "loggingmacros.h"

#define     THIS_PACKAGE    "stem_tracker"
#define     THIS_NODE       THIS_PACKAGE


int main(int argc, char** argv)
{
    /* initialize profiling */
    StatsPublisher sp;

    /* initialize configuration */
    StemTrackConfigurer TomatoConfigurer;
    TomatoConfigurer.loadConfig(argc, argv, (ros::package::getPath(THIS_PACKAGE) + "/config/"), "config.yml" );
    if(!TomatoConfigurer.configIsOk())
        return 1;

    /* initialize node */
    ros::init(argc, argv, THIS_NODE);
    ros::NodeHandle n;
    ros::Rate r(TomatoConfigurer.getUpdateRate());

    /* initialize and configure stem represenation object */
    StemRepresentation TomatoStem(1);
    TomatoConfigurer.configureStemRepresentation(TomatoStem);

    /* initialize and configure robot representation object */
    RobotRepresentation AmigoRepresentation("amigo");
    TomatoConfigurer.configureRobotRepresentation(AmigoRepresentation);

    /* initialize and configure robot status object */
    RobotStatus AmigoStatus(&AmigoRepresentation);
    TomatoConfigurer.configureRobotStatus(AmigoStatus);

    /* initialize and configure stem tracking controller object */
    StemTrackController TomatoControl(&AmigoRepresentation, &AmigoStatus, &TomatoStem);
    TomatoConfigurer.configureStemTrackController(TomatoControl);

    /* initialize and configure whisker interpretation object */
    WhiskerGripperInterpreter TomatoWhiskerGripper(&AmigoStatus, &TomatoControl);
    TomatoConfigurer.configureWhiskerGripperInterpreter(TomatoWhiskerGripper);

    /* initialize and configure interface to robot object */
    RobotInterface AmigoInterface(n, &AmigoRepresentation, &AmigoStatus);
    TomatoConfigurer.configureRobotInterface(AmigoInterface);

    /* initialize and configure visualization object */
    VisualizationInterface RvizInterface(n, TomatoConfigurer.getBaseFrame());
    TomatoConfigurer.configureVisualizationInterface(RvizInterface);

    /* initialize state machine and safety monitor */
    StemTrackMonitor TomatoMonitor(&TomatoStem, &AmigoRepresentation, &AmigoStatus, &TomatoControl, &TomatoWhiskerGripper,
                                   &TomatoConfigurer, &AmigoInterface, &RvizInterface);
    TomatoConfigurer.configureStemTrackMonitor(TomatoMonitor);

    /* initialize profiling */
    sp.initialize();

    /* ------------------
       main update loop */

    while(ros::ok())
    {
        /* start timer, for profiling */
        sp.startTimer("main");

        /* update tomato stemtracking */
        if(!TomatoMonitor.update())
        {
            ERROR_STREAM("Error in TomatoMonitor update!");
            return 1;
        }

        /* stop and publish timer */
        sp.stopTimer("main");
        sp.publish();

        /* wait for next sample */
        r.sleep();
        ros::spinOnce();

    }

    return 0;
}
