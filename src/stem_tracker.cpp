#include "stem_tracker.h"

// TODO:
//- load urdf from file, and make filename configurable, than n is not needed anymore in constructor of robotrepresentation
//- slimmere pose target, niet door stem heen en met helling meedraaien
//- reageren op combinatie van whisker forces ipv naar bekende intersection



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
    StemTrackController TomatoControl(&AmigoRepresentation, &AmigoStatus);
    StemTrackConfigurer.configureStemTrackController(config, &TomatoControl);

    /* initialize state machine and safety monitor */
    StemTrackMonitor TomatoMonitor(&TomatoStem);

    /* initialize and configure visualization object */
    VisualizationInterface RvizInterface(n, StemTrackConfigurer.getBaseFrame(config));

    /* initialize interface to robot object */
    RobotInterface AmigoInterface(n, &AmigoRepresentation, &AmigoStatus);

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
        }

        if (!config.hasError())
        {
            /* visualize stem */
            RvizInterface.showLineStrip(TomatoStem.getNodesX(), TomatoStem.getNodesY(), TomatoStem.getNodesZ(), stem);

            /* start timer, for profiling */
            sp.startTimer("main");

            if(initializing && AmigoStatus.isUpToDate())
            {        
                /* bring arm to initial position */
                AmigoInterface.publishAmigoArmMessage(AmigoRepresentation.getAmigoInitialPoseMsg());
                initializing = false;
                INFO_STREAM("===========================");
                INFO_STREAM("Ready for stem tracking");
                INFO_STREAM("===========================");
            }

            if (!initializing && AmigoStatus.hasValidGripperXYZ() )
            {
                /* forward kinematics */
                RvizInterface.showXYZ(AmigoStatus.getGripperXYZ(), gripper_center);

                /* find and show nearest intersection with stem */
                TomatoStem.updateNearestXYZ(AmigoStatus.getGripperXYZ());
                RvizInterface.showXYZ(TomatoStem.getNearestXYZ(), nearest_stem_intersection);

                /* simulate and show whisker sensor output */
                TomatoWhiskerGripper.simulateWhiskerGripper(AmigoStatus.getGripperXYZ(), TomatoStem.getNearestXYZ() );
                RvizInterface.showForce(TomatoWhiskerGripper.getWhiskerNetForce(), AmigoStatus.getGripperXYZ(), whisker_net_force);

                /* check have we reached end of stem */
                if (TomatoMonitor.reachedEndOfStem(up))
                    up = -up;

                /* update position setpoint in cartesian space */
                TomatoControl.updateCartSetpoint(AmigoStatus.getGripperXYZ(), TomatoWhiskerGripper.getXYerror(), up);

                /* translate cartesian setpoint to joint coordinates */
                TomatoControl.updateJointReferences();

                /* send references to joint controllers */
                AmigoInterface.publishJointPosRefs(TomatoControl.getJointRefs());
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
