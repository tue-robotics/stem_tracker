#include "stem_tracker.h"

// TODO:
//- slimmere pose target, met helling meedraaien
//- alleen velocity solver in de loop ipv telkens complete positie ik
//- preposition om te voorkomen dat door de stem heen
//- check of preposition behaald
//- reageren op combinatie van whisker forces ipv naar bekende intersection
//- check voor welke objecten interface naar andere object alleen voor config nodig is (vb in robotstatus)
//- maak een robotstatus up to date reset, bijv om na wisselen van arm opnieuw to update te wachten
//- robotstatus up to date check voor volledige array to monitor
//- profilen (sample rate omhoog om op amigo te testen)

int main(int argc, char** argv)
{

    /* initialize state handling params */
    bool initializing = true;
    int up = 1;

    /* initialize profiling object */
    StatsPublisher sp;

    /* initialize configuration */
    tue::Configuration config;
    Configurer StemTrackConfigurer;

    /* load configuration */
    if (argc >= 2)
    {
        std::string yaml_filename = argv[1];
        config.loadFromYAMLFile(yaml_filename);
    }
    else
    {
        std::string this_package_dir = ros::package::getPath(THIS_PACKAGE);
        config.loadFromYAMLFile(this_package_dir + "/config/config.yml");
    }

    StemTrackConfigurer.loadGlobalConfig(config);

    if (config.hasError())
    {
        INFO_STREAM("Could not load configuration: " << config.error());
        return 1;
    }

    /* initialize node */
    ros::init(argc, argv, THIS_PACKAGE);
    ros::NodeHandle n;
    ros::Rate r(StemTrackConfigurer.getUpdateRate(config));

    /* initialize and configure stem represenation object */
    StemRepresentation TomatoStem(1);
    StemTrackConfigurer.configureStemRepresentation(config, &TomatoStem);

    /* initialize and configure robot representation object */
    RobotRepresentation AmigoRepresentation("amigo");
    StemTrackConfigurer.configureRobotRepresentation(config, &AmigoRepresentation, n);

    /* initialize and configure whisker interpretation object */
    WhiskerInterpreter TomatoWhiskerGripper(1);
    StemTrackConfigurer.configureWhiskerInterpreter(config, &TomatoWhiskerGripper);

    /* initialize and configure robot status object */
    RobotStatus AmigoStatus(&AmigoRepresentation);
    StemTrackConfigurer.configureRobotStatus(config, &AmigoStatus);

    /* initialize and configure stem tracking controller object */
    StemTrackController TomatoControl(&AmigoRepresentation, &AmigoStatus, &TomatoStem);
    StemTrackConfigurer.configureStemTrackController(config, &TomatoControl);

    /* initialize state machine and safety monitor */
    StemTrackMonitor TomatoMonitor(&TomatoStem);

    /* initialize and configure visualization object */
    VisualizationInterface RvizInterface(n, StemTrackConfigurer.getBaseFrame(config));

    /* initialize interface to robot object */
    RobotInterface AmigoInterface(n, &AmigoRepresentation, &AmigoStatus);
    StemTrackConfigurer.configureRobotInterface(config, &AmigoInterface);

    /* initialize profiling */
    sp.initialize();

    /* main update loop */
    while(ros::ok())
    {

        if (config.sync())
        {
            /* config file changed */
            StemTrackConfigurer.loadGlobalConfig(config);
            StemTrackConfigurer.configureStemRepresentation(config, &TomatoStem);
            StemTrackConfigurer.configureRobotRepresentation(config, &AmigoRepresentation, n);
            StemTrackConfigurer.configureRobotStatus(config, &AmigoStatus);
            StemTrackConfigurer.configureWhiskerInterpreter(config, &TomatoWhiskerGripper);
            StemTrackConfigurer.configureStemTrackController(config, &TomatoControl);
            StemTrackConfigurer.configureRobotInterface(config, &AmigoInterface);
        }

        if (!config.hasError())
        {
            /* visualize stem */
            RvizInterface.showLineStrip(TomatoStem.getNodesX(), TomatoStem.getNodesY(), TomatoStem.getNodesZ(), stem);

            /* start timer, for profiling */
            sp.startTimer("main");

            if(TomatoMonitor.getState() == STEMTRACK_PREPOS && AmigoStatus.isUpToDate())
            {        
                /* bring arm to initial position */
                AmigoInterface.publishJointPosRefs(AmigoRepresentation.getInitialPoseJointRefs());

                TomatoMonitor.updateState();
            }

            if (TomatoMonitor.getState() == STEMTRACK_FOLLOW && AmigoStatus.hasValidGripperXYZ() )
            {
                /* forward kinematics */
                RvizInterface.showXYZ(AmigoStatus.getGripperXYZ(), gripper_center);

                /* find and show nearest intersection with stem */
                TomatoStem.updateNearestXYZ(AmigoStatus.getGripperXYZ());
                RvizInterface.showXYZ(TomatoStem.getNearestXYZ(), nearest_stem_intersection);

                /* simulate and show whisker sensor output */
                TomatoWhiskerGripper.simulateWhiskerGripper(AmigoStatus.getGripperXYZ(), TomatoStem.getNearestXYZ() );
                RvizInterface.showForce(TomatoWhiskerGripper.getWhiskerNetForce(), AmigoStatus.getGripperXYZ(), whisker_net_force);

                /* update position setpoint in cartesian space */
                TomatoControl.updateCartSetpoint(AmigoStatus.getGripperXYZ(), TomatoWhiskerGripper.getXYerror(), up);

                /* translate cartesian setpoint to joint coordinates */
                TomatoControl.updateJointReferences();

                /* send references to joint controllers */
                AmigoInterface.publishJointPosRefs(TomatoControl.getJointRefs());

                TomatoMonitor.updateState();
            }

            if(!AmigoStatus.isUpToDate())
                INFO_STREAM("waiting for up to date robot status information");

            /* stop and publish timer */
            sp.stopTimer("main");
            sp.publish();

        }

        else
            INFO_STREAM("error in loading config file!");

        /* wait for next sample */
        r.sleep();
        ros::spinOnce();

    }

    return 0;
}
