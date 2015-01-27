#include "stem_tracker.h"

// TODO:
//- rekening houden met joint limits als alleen ik vel solver, nulruimte term toevoegen
//- tilt met stem fixen als alleen ik vel solver
//- template ipv extractFloat, extractDouble etc
//- z verplaatsing functie van error
//- reageren op combinatie van whisker forces ipv naar bekende intersection
//- check voor welke objecten interface naar andere object alleen voor config nodig is (vb in robotstatus)
//- maak een robotstatus up to date reset, bijv om na wisselen van arm opnieuw op up to date info te wachten
//- robotstatus up to date check voor volledige array
//- selfchecks updaten of weghalen
//- use stem length instead of distance in z for lin_tan_d
//- veiligheidscheck voor verspringen joint coordinaten
//- use z rotation of setpoint to increase reachable space
//- orientatie base frame tov gripper frame voor 'neutrale' pose configureerbaar maken

// KNOWN-BUGS
//- hangen bij hele lage z-snelheid ref


int main(int argc, char** argv)
{

    /* initialize profiling */
    StatsPublisher sp;

    /* initialize configuration */
    tue::Configuration config;
    StemTrackConfigurer TomatoConfigurer;

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

    if (config.hasError())
    {
        ERROR_STREAM("Could not load configuration: " << config.error());
        return 1;
    }

    /* initialize node */
    ros::init(argc, argv, THIS_PACKAGE);
    ros::NodeHandle n;
    ros::Rate r(TomatoConfigurer.getUpdateRate(config));

    /* initialize and configure stem represenation object */
    StemRepresentation TomatoStem(1);
    TomatoConfigurer.configureStemRepresentation(config, TomatoStem);

    /* initialize and configure robot representation object */
    RobotRepresentation AmigoRepresentation("amigo");
    TomatoConfigurer.configureRobotRepresentation(config, AmigoRepresentation);

    /* initialize and configure whisker interpretation object */
    WhiskerInterpreter TomatoWhiskerGripper(&AmigoRepresentation);
    TomatoConfigurer.configureWhiskerInterpreter(config, TomatoWhiskerGripper);

    /* initialize and configure robot status object */
    RobotStatus AmigoStatus(&AmigoRepresentation);
    TomatoConfigurer.configureRobotStatus(config, AmigoStatus);

    /* initialize and configure stem tracking controller object */
    StemTrackController TomatoControl(&AmigoRepresentation, &AmigoStatus, &TomatoStem);
    TomatoConfigurer.configureStemTrackController(config, TomatoControl);

    /* initialize interface to robot object */
    RobotInterface AmigoInterface(n, &AmigoRepresentation, &AmigoStatus);
    TomatoConfigurer.configureRobotInterface(config, AmigoInterface);

    /* initialize state machine and safety monitor */
    StemTrackMonitor TomatoMonitor(&TomatoStem, &AmigoRepresentation, &AmigoStatus, &TomatoControl);
    TomatoConfigurer.configureStemTrackMonitor(config, TomatoMonitor);

    /* initialize and configure visualization object */
    VisualizationInterface RvizInterface(n, TomatoConfigurer.getBaseFrame(config));
    TomatoConfigurer.configureVisualizationInterface(config, RvizInterface);

    /* initialize profiling */
    sp.initialize();



    /* main update loop */
    while(ros::ok())
    {

        if (config.sync())
        {
            /* config file changed */
            TomatoConfigurer.configureStemRepresentation(config, TomatoStem);
            TomatoConfigurer.configureRobotRepresentation(config, AmigoRepresentation);
            TomatoConfigurer.configureRobotStatus(config, AmigoStatus);
            TomatoConfigurer.configureWhiskerInterpreter(config, TomatoWhiskerGripper);
            TomatoConfigurer.configureStemTrackController(config, TomatoControl);
            TomatoConfigurer.configureRobotInterface(config, AmigoInterface);
            TomatoConfigurer.configureStemTrackMonitor(config, TomatoMonitor);
            TomatoConfigurer.configureVisualizationInterface(config, RvizInterface);
        }

        if (!config.hasError())
        {
            /* visualize stem */
            RvizInterface.showLineStrip(TomatoStem.getNodesX(), TomatoStem.getNodesY(), TomatoStem.getNodesZ(), stem);

            /* start timer, for profiling */
            sp.startTimer("main");


            if(TomatoMonitor.getState() == STEMTRACK_STATE_PREPOS && AmigoStatus.isUpToDate())
            {
                /* bring arm to initial position */
                AmigoInterface.publishJointPosRefs(AmigoRepresentation.getInitialPoseJointRefs());
                TomatoMonitor.updateState();
            }

            if(TomatoMonitor.getState() == STEMTRACK_STATE_GRASP  )
            {
                /* find and show nearest intersection with stem */
                TomatoStem.updateNearestXYZ(AmigoStatus.getGripperXYZ());
                RvizInterface.showXYZ(TomatoStem.getNearestXYZ(), nearest_stem_intersection);

                /* set reference to nearest stem intersection */
                TomatoControl.updateCartSetpoint(TomatoStem.getNearestXYZ());

                /* translate cartesian setpoint to joint coordinates */
                TomatoControl.updateJointPosReferences();

                /* send references to joint controllers */
                AmigoInterface.publishJointPosRefs(TomatoControl.getJointPosRefs());

                /* check if reference reached */
                TomatoMonitor.updateState();
            }

            if(TomatoMonitor.getState() == STEMTRACK_STATE_FOLLOW && AmigoStatus.hasValidGripperXYZ() )
            {
                /* forward kinematics */
                RvizInterface.showXYZ(AmigoStatus.getGripperXYZ(), gripper_center);

                /* find and show nearest intersection with stem */
                TomatoStem.updateNearestXYZ(AmigoStatus.getGripperXYZ());
                RvizInterface.showXYZ(TomatoStem.getNearestXYZ(), nearest_stem_intersection);

                /* simulate and show whisker sensor output */
                TomatoWhiskerGripper.simulateWhiskerGripper(AmigoStatus.getGripperXYZ(), TomatoStem.getNearestXYZ() );
//                RvizInterface.showArrow(TomatoWhiskerGripper.getWhiskerNetForce(), AmigoStatus.getGripperXYZ(), whisker_net_force);

                /* update position setpoint in cartesian space */
                TomatoControl.updateCartSetpoint(AmigoStatus.getGripperXYZ(), TomatoWhiskerGripper.getEstimatedPosError());
                RvizInterface.showArrow(TomatoStem.getCurrentTangent(), TomatoStem.getNearestXYZ(), stem_tangent);

                /* translate cartesian setpoint to joint coordinates */
                TomatoControl.updateJointPosReferences();

                /* send references to joint controllers */
                AmigoInterface.publishJointPosRefs(TomatoControl.getJointPosRefs());

                /* check if end of stem reached */
                TomatoMonitor.updateState();
            }

            if(TomatoMonitor.getState() == STEMTRACK_STATE_END)
            {
                TomatoWhiskerGripper.readWhiskers();
                RvizInterface.showArrow(TomatoWhiskerGripper.getEstimatedPosError(), AmigoStatus.getGripperXYZ(), whisker_net_force);
            }

            if(!AmigoStatus.isUpToDate())
                INFO_STREAM("waiting for up to date robot status information");

            /* stop and publish timer */
            sp.stopTimer("main");
            sp.publish();

        }

        else
            ERROR_STREAM("error in loading config file!");

        /* wait for next sample */
        r.sleep();
        ros::spinOnce();

    }

    return 0;
}
