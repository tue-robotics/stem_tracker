#include "stem_tracker.h"

// TODO:
//- naar beginpunt als boven of onder stem in z
//- niet alle config globaal beschikbaar
//- check maken voor wat er precies veranderd is ipv hele config opnieuw
//- slimmere pose target, niet door stem heen en met helling meedraaien
//- reageren op combinatie van whisker forces ipv naar bekende intersection



void loadConfiguration(tue::Configuration config)
{

    INFO_STREAM("================");
    INFO_STREAM("(Re)Configuring");

    /* general configuration */
    config.value("base_frame", BASE_FRAME);
    INFO_STREAM("base_frame = " << BASE_FRAME);
    config.value("debug", DEBUG);
    INFO_STREAM("debug = " << DEBUG);
    config.value("update_rate", UPDATE_RATE);
    INFO_STREAM("update_rate = " << UPDATE_RATE);

    /* robot configuration */
    config.value("robot_name", ROBOT_NAME);
    config.value("root_link", ROOT_LINK);
    config.value("use_leftarm", USE_LEFTARM);
    config.value("robot_description_rosparam", ROBOT_DESCRIPTION_ROSPARAM);
    config.value("left_end_link", LEFT_END_LINK);
    config.value("right_end_link", RIGHT_END_LINK);
    config.value("up_to_date_threshold", UP_TO_DATE_THRESHOLD);
    INFO_STREAM("up_to_date_threshold = " << UP_TO_DATE_THRESHOLD);

    /* whisker configuration */
    config.value("n_whiskers", N_WHISKERS);
    config.value("whisker_length", WHISKER_LENGTH);
    config.value("gripper_diameter", GRIPPER_DIAMETER);

    /* tomato stem configuration */
    if (config.readArray("stem_nodes"))
    {
        float tmp;
        stemNodesX.clear(); stemNodesY.clear(); stemNodesZ.clear();

        while(config.nextArrayItem())
        {
            config.value("x", tmp); stemNodesX.push_back(tmp);
            config.value("y", tmp); stemNodesY.push_back(tmp);
            config.value("z", tmp); stemNodesZ.push_back(tmp);
        }

        config.endArray();
    }

    /* stem tracking control configuration */
    config.value("max_z_dot", MAX_Z_DOT);
}

void initStem(StemRepresentation* stem)
{
    stem->loadNodesXYZ(stemNodesX, stemNodesY, stemNodesZ);
    if(!USE_LEFTARM)
        stem->flipNodes();
    if(DEBUG)
        stem->printAll();
}

void initRobotConfig(RobotConfig* robot_config, ros::NodeHandle n)
{
    if(USE_LEFTARM)
        robot_config->setLeftArmIsPreferred();
    else
        robot_config->setRightArmIsPreferred();

    robot_config->loadUrdfFromRosparam(n, ROBOT_DESCRIPTION_ROSPARAM);
    robot_config->loadKinematicTreeFromUrdf();

    if(USE_LEFTARM)
        robot_config->loadKinematicChainFromTree(ROOT_LINK, LEFT_END_LINK);
    else
        robot_config->loadKinematicChainFromTree(ROOT_LINK, RIGHT_END_LINK);

    robot_config->loadJointLimits();

    if(DEBUG)
        robot_config->printAll();
}

void initRobotStatus(RobotStatus* robot_status)
{
    robot_status->setUpToDateThreshold( UP_TO_DATE_THRESHOLD );
}

int main(int argc, char** argv)
{

    /* initialize state handling params */

    bool initializing = true;
    int up = 1;

    /* initialize profiling object */

    StatsPublisher sp;

    /* load yaml config file */

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

    loadConfiguration(config);

    if (config.hasError())
    {
        INFO_STREAM("Could not load configuration: " << config.error());
        return 1;
    }

    /* initialize node */

    ros::init(argc, argv, THIS_PACKAGE);
    ros::NodeHandle n;
    ros::Rate r(UPDATE_RATE);

    /* initialize stem represenation object */

    StemRepresentation TomatoStem(1);
    initStem(&TomatoStem);

    /* initialize robot configuration object */

    RobotConfig AmigoConfig(ROBOT_NAME);
    initRobotConfig(&AmigoConfig, n);

    /* initialize whisker interpretation object */

    WhiskerInterpreter TomatoWhiskerGripper(N_WHISKERS, 1, WHISKER_LENGTH, GRIPPER_DIAMETER);

    /* initialize robot status object */

    RobotStatus AmigoStatus(AmigoConfig.getKinematicChain().getNrOfJoints(), &AmigoConfig);
    initRobotStatus(&AmigoStatus);

    /* initialize interface to robot object */
    RobotInterface AmigoInterface(n, &AmigoConfig, &AmigoStatus);

    /* initialize stem tracking controller object */
    StemTrackController TomatoControl(MAX_Z_DOT, UPDATE_RATE, &AmigoConfig, &AmigoStatus);

    /* initialize state machine and safety monitor */
    StemTrackMonitor TomatoMonitor(&TomatoStem);

    /* initialize visualization object */
    VisualizationInterface RvizInterface(n, BASE_FRAME);

    /* initialize profiling */
    sp.initialize();

    /* main update loop */
    while(ros::ok())
    {

        if (config.sync())
        {
            /* config file changed */
            loadConfiguration(config);
            initStem(&TomatoStem);
            initRobotConfig(&AmigoConfig, n);
            initRobotStatus(&AmigoStatus);
        }

        if (!config.hasError())
        {
            /* visualize stem */
            RvizInterface.showLineStrip(TomatoStem.getNodesX(), TomatoStem.getNodesY(), TomatoStem.getNodesZ(), stem);

            /* start sample timing, for profiling */
            sp.startTimer("main");

            if(initializing && AmigoStatus.isUpToDate())
            {
                /* bring arm to initial position */
                AmigoInterface.publishAmigoArmMessage(AmigoConfig.getAmigoInitialPoseMsg());
                initializing = false;
                INFO_STREAM("=============");
                INFO_STREAM("Initialized");
                INFO_STREAM("=============");
            }

            if (!initializing && AmigoStatus.hasValidGripperXYZ() )
            {

                /* forward kinematics */
                RvizInterface.showXYZ(AmigoStatus.getGripperXYZ(), gripper_center);

                /* find and show nearest coordinate on stem */
                TomatoStem.updateNearestXYZ(AmigoStatus.getGripperXYZ());
                RvizInterface.showXYZ(TomatoStem.getNearestXYZ(), nearest_stem_intersection);

                /* simulate and show whiskers */
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
                AmigoInterface.publishJointPosRefs( TomatoControl.getJointRefs());

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
};
