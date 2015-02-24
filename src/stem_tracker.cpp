#include "stem_tracker.h"

// TODO:
//- reageren op combinatie van whisker forces ipv naar bekende intersection
//- rekening houden met joint limits als alleen ik vel solver, nulruimte term toevoegen
//- tilt met stem fixen als alleen ik vel solver
//- z verplaatsing functie van error
//- whisker orocos component alleen sturen als msg ontvangen
//- check voor welke objecten interface naar andere object alleen voor config nodig is (vb in robotstatus)
//- use stem length instead of distance in z for lin_tan_d
//- use rotation around z to increase reachable space
//- ints ipv array voor analoginsgeneric

// KNOWN-BUGS
//- hangen bij hele lage z-snelheid ref
//- in config: een spatie na een int werkt wel maar een tab na een int zorgt ervoor dat type niet meer herkend

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
    ros::init(argc, argv, THIS_NODE);
    ros::NodeHandle n;
    ros::Rate r(TomatoConfigurer.getUpdateRate(config));

    /* initialize and configure stem represenation object */
    StemRepresentation TomatoStem(1);
    TomatoConfigurer.configureStemRepresentation(config, TomatoStem);

    /* initialize and configure robot representation object */
    RobotRepresentation AmigoRepresentation("amigo");
    TomatoConfigurer.configureRobotRepresentation(config, AmigoRepresentation);

    /* initialize and configure robot status object */
    RobotStatus AmigoStatus(&AmigoRepresentation);
    TomatoConfigurer.configureRobotStatus(config, AmigoStatus);

    /* initialize and configure whisker interpretation object */
    WhiskerInterpreter TomatoWhiskerGripper(&AmigoStatus);
    TomatoConfigurer.configureWhiskerInterpreter(config, TomatoWhiskerGripper);

    /* initialize and configure stem tracking controller object */
    StemTrackController TomatoControl(&AmigoRepresentation, &AmigoStatus, &TomatoStem);
    TomatoConfigurer.configureStemTrackController(config, TomatoControl);

    /* initialize interface to robot object */
    RobotInterface AmigoInterface(n, &AmigoRepresentation, &AmigoStatus);
    TomatoConfigurer.configureRobotInterface(config, AmigoInterface);

    /* initialize state machine and safety monitor */
    StemTrackMonitor TomatoMonitor(&TomatoStem, &AmigoRepresentation, &AmigoStatus, &TomatoControl, &TomatoWhiskerGripper);
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
            AmigoStatus.resetUpToDateStatus();
        }

        if (!config.hasError())
        {
            /* visualize stem */
            RvizInterface.showLineStrip(TomatoStem.getNodesX(), TomatoStem.getNodesY(), TomatoStem.getNodesZ(), stem);

            /* start timer, for profiling */
            sp.startTimer("main");

            /* obtain nominal whisker values */
            if(TomatoMonitor.getState() == CALIBRATE && AmigoStatus.whiskerMeasurementsAreUpToDate() && AmigoStatus.pressureSensorMeasurementsAreUpToDate())
            {
                TomatoWhiskerGripper.obtainNominalValues();
                TomatoMonitor.updateState();
            }

            /* bring arm to preposition */
            if(TomatoMonitor.getState() == PREPOS && AmigoStatus.jointStatusIsUpToDate())
            {
                AmigoInterface.publishAmigoJointPosRefs(AmigoRepresentation.getInitialPoseJointRefs());
                TomatoMonitor.updateState();
            }

            if(TomatoMonitor.getState() == GRASP && AmigoStatus.isUpToDate()  )
            {
                /* find and show nearest intersection with stem */
                TomatoStem.updateNearestXYZ(AmigoStatus.getGripperXYZ());
                RvizInterface.showXYZ(TomatoStem.getNearestXYZ(), nearest_stem_intersection);

                /* set reference to nearest stem intersection */
                TomatoControl.updateCartSetpoint(TomatoStem.getNearestXYZ());

                /* translate cartesian setpoint to joint coordinates */
                TomatoControl.updateJointPosReferences();

                /* send references to joint controllers */
                AmigoInterface.publishAmigoJointPosRefs(TomatoControl.getJointPosRefs());

                /* check if reference reached */
                TomatoMonitor.updateState();
            }

            if(TomatoMonitor.getState() == FOLLOW && AmigoStatus.hasValidGripperXYZ() && AmigoStatus.isUpToDate() )
            {
                /* forward kinematics */
                RvizInterface.showXYZ(AmigoStatus.getGripperXYZ(), gripper_center);

                /* find and show nearest intersection with stem */
                TomatoStem.updateNearestXYZ(AmigoStatus.getGripperXYZ());
                RvizInterface.showXYZ(TomatoStem.getNearestXYZ(), nearest_stem_intersection);

                /* simulate and show whisker sensor output */
                TomatoWhiskerGripper.simulateWhiskerGripper(AmigoStatus.getGripperXYZ(), TomatoStem.getNearestXYZ() );

                /* update position setpoint in cartesian space */
                TomatoControl.updateCartSetpoint(AmigoStatus.getGripperXYZ(), TomatoWhiskerGripper.getEstimatedPosError());
                RvizInterface.showArrow(TomatoStem.getCurrentTangent(), TomatoStem.getNearestXYZ(), stem_tangent);

                /* translate cartesian setpoint to joint coordinates */
                TomatoControl.updateJointPosReferences();

                /* send references to joint controllers */
                AmigoInterface.publishAmigoJointPosRefs(TomatoControl.getJointPosRefs());

                /* check if end of stem reached */
                TomatoMonitor.updateState();
            }

            if(TomatoMonitor.getState() == END)
            {
                TomatoWhiskerGripper.readWhiskers();
                RvizInterface.showArrows(TomatoWhiskerGripper.getWhiskersTouchedAt(), 0.005, 0.03, AmigoStatus.getGripperXYZ(), whisker_touch);
                printVector(TomatoWhiskerGripper.getWhiskersTouchedAt());

            }

            if(!AmigoStatus.jointStatusIsUpToDate())
                INFO_STREAM("Waiting for up to date joint status information");

            if(!AmigoStatus.whiskerMeasurementsAreUpToDate() || !AmigoStatus.pressureSensorMeasurementsAreUpToDate())
                INFO_STREAM("Waiting for up to date whiskergripper status information");

            /* stop and publish timer */
            sp.stopTimer("main");
            sp.publish();

        }

        else
            ERROR_STREAM("Error in config file: " << config.error());

        /* wait for next sample */
        r.sleep();
        ros::spinOnce();

    }

    return 0;
}
