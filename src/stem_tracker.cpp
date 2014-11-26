#include "stem_tracker.h"

// TODO:
//- rekening houden met joint limits als alleen ik vel solver
//- tilt met stem fixen als alleen ik vel solver
//- z verplaatsing functie van error
//- reageren op combinatie van whisker forces ipv naar bekende intersection
//- orientatie base frame tov gripper frame voor 'neutrale' pose configureerbaar maken
//- check voor welke objecten interface naar andere object alleen voor config nodig is (vb in robotstatus)
//- maak een robotstatus up to date reset, bijv om na wisselen van arm opnieuw op up to date info te wachten
//- robotstatus up to date check voor volledige array
//- selfchecks updaten of weghalen
//- check voor loslaten van stengel
//- check voor tegenkomen side branch
//- use stem length instead of distance in z for lin_tan_d
//- veiligheidscheck voor verspringen joint coordinaten
//- waarom hangen bij hele lage snelheid
//- INFO STREAM niet in alle nodes, printall stringstream laten returnen


int main(int argc, char** argv)
{

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

    /* initialize interface to robot object */
    RobotInterface AmigoInterface(n, &AmigoRepresentation, &AmigoStatus);
    StemTrackConfigurer.configureRobotInterface(config, &AmigoInterface);

    /* initialize state machine and safety monitor */
    StemTrackMonitor TomatoMonitor(&TomatoStem, &AmigoRepresentation, &AmigoStatus, &TomatoControl);
    StemTrackConfigurer.configureStemTrackMonitor(config, &TomatoMonitor);

    /* initialize and configure visualization object */
    VisualizationInterface RvizInterface(n, StemTrackConfigurer.getBaseFrame(config));

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
            StemTrackConfigurer.configureStemTrackMonitor(config, &TomatoMonitor);
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

            if(TomatoMonitor.getState() == STEMTRACK_GRASP  )
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

            if(TomatoMonitor.getState() == STEMTRACK_FOLLOW && AmigoStatus.hasValidGripperXYZ() )
            {
                /* forward kinematics */
                RvizInterface.showXYZ(AmigoStatus.getGripperXYZ(), gripper_center);

                /* find and show nearest intersection with stem */
                TomatoStem.updateNearestXYZ(AmigoStatus.getGripperXYZ());
                RvizInterface.showXYZ(TomatoStem.getNearestXYZ(), nearest_stem_intersection);


                /* simulate and show whisker sensor output */
                TomatoWhiskerGripper.simulateWhiskerGripper(AmigoStatus.getGripperXYZ(), TomatoStem.getNearestXYZ() );
                RvizInterface.showArrow(TomatoWhiskerGripper.getWhiskerNetForce(), AmigoStatus.getGripperXYZ(), whisker_net_force);

                /* update position setpoint in cartesian space */
                TomatoControl.updateCartSetpoint(AmigoStatus.getGripperXYZ(), TomatoWhiskerGripper.getXYerror());
                RvizInterface.showArrow(TomatoStem.getCurrentTangent(), TomatoStem.getNearestXYZ(), stem_tangent);

                /* translate cartesian setpoint to joint coordinates */
                TomatoControl.updateJointPosReferences();

                /* send references to joint controllers */
                AmigoInterface.publishJointPosRefs(TomatoControl.getJointPosRefs());

                /* check if end of stem reached */
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
