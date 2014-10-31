#include "stem_tracker.h"

// TODO:
//- robot interface object
//- naar beginpunt als boven of onder stem in z
//- niet alle config globaal beschikbaar
//- check maken voor wat er precies veranderd is ipv hele config opnieuw
//- debug functies apart, verschillende loglevels maken
//- slimmere pose target, niet door stem heen en met helling meedraaien
//- reageren op whisker force ipv naar bekende intersection
//- stem track controller object maken



void loadConfiguration(tue::Configuration config)
{

    INFO_STREAM("================");
    INFO_STREAM("(Re)Configuring");

    /* general configuration */
    config.value("base_frame", BASE_FRAME);
    INFO_STREAM("base_frame = " << BASE_FRAME);
    config.value("debug", DEBUG);
    INFO_STREAM("debug = " << DEBUG);
    config.value("debug_ik", DEBUG_IK);
    INFO_STREAM("debug_ik = " << DEBUG_IK);
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

void printKDLframe(KDL::Frame kdl_frame)
{
    std::stringstream frame_stream;

    for(int i=0; i<4; ++i)
    {
        frame_stream.str(""); frame_stream << std::scientific;

        for(int j=0; j<4; ++j)
            frame_stream << kdl_frame(i,j) << "\t";

        INFO_STREAM(frame_stream.str());
    }
}

void printXYZvector(std::vector<float> vect)
{
    INFO_STREAM("x = " << vect.at(0) << " y = " << vect.at(1) << " z = " << vect.at(2));
}


int main(int argc, char** argv)
{

    /* initialize state handling params */

    bool initializing = true;
    int up = 1;

    /* declare communication objects */

    ros::Publisher visualization_publisher;
    ros::Publisher arm_reference_publisher;
    ros::Publisher torso_references_publisher;
    ros::Subscriber arm_measurements_subscriber;
    ros::Subscriber torso_measurements_subscriber;

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

    /* initialize visualization via rviz */
    visualization_publisher = n.advertise<visualization_msgs::Marker>( "visualization_marker", 100 );
    VisualizationInterface StemTrackerInRviz(&visualization_publisher, BASE_FRAME);

    /* initialize node communication */

    if (USE_LEFTARM)
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 0);
    else
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 0);

    torso_measurements_subscriber = n.subscribe("/amigo/torso/measurements", 1000, &RobotStatus::receivedTorsoMsg, &AmigoStatus);
    torso_references_publisher = n.advertise<sensor_msgs::JointState>("/amigo/torso/references", 0);

    if (USE_LEFTARM)
        arm_measurements_subscriber = n.subscribe("/amigo/left_arm/measurements", 1000, &RobotStatus::receivedArmMsg, &AmigoStatus);
    else
        arm_measurements_subscriber = n.subscribe("/amigo/right_arm/measurements", 1000, &RobotStatus::receivedArmMsg, &AmigoStatus);

    /* initialize profiling */
    sp.initialize();

    /* update loop */
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
            StemTrackerInRviz.showLineStrip(TomatoStem.getNodesX(), TomatoStem.getNodesY(), TomatoStem.getNodesZ(), stem);

            /* start sample timing, for profiling */
            sp.startTimer("main");

            if(initializing && AmigoStatus.isUpToDate())
            {
                /* bring arm to initial position */
                arm_reference_publisher.publish(AmigoConfig.getAmigoInitialPoseMsg());
                initializing = false;
                INFO_STREAM("=============");
                INFO_STREAM("Initialized");
                INFO_STREAM("=============");
            }

            if (!initializing && AmigoStatus.hasValidGripperXYZ() )
            {

                /* forward kinematics */
                StemTrackerInRviz.showXYZ(AmigoStatus.getGripperXYZ(), gripper_center);

                /* nearest coordinate on stem */
                TomatoStem.getNearestXYZonStem(AmigoStatus.getGripperXYZ());

                /* obtain simulated whisker measurement */
                TomatoWhiskerGripper.simulateWhiskerGripper(AmigoStatus.getGripperXYZ(), TomatoStem.getNearestXYZonStem(AmigoStatus.getGripperXYZ()) );
                StemTrackerInRviz.showForce(TomatoWhiskerGripper.getWhiskerNetForce(), AmigoStatus.getGripperXYZ(), whisker_net_force);

                /* calculate new cartesian setpoint */
                StemTrackerInRviz.showXYZ(TomatoStem.getNearestXYZonStem( AmigoStatus.getGripperXYZ() ), nearest_stem_intersection);

                //                StemTrackController.updateFeedforward();
                //                StemTrackController.updateSetpoint();
                //                RobotInterface.publishJointSetpoints();

                //===================================================================================================================================

                std::vector<float> gripper_xyz, stem_intersection_xyz;

                gripper_xyz = AmigoStatus.getGripperXYZ();
                stem_intersection_xyz = TomatoStem.getNearestXYZonStem(gripper_xyz);

                //=========================================

                boost::shared_ptr<KDL::ChainFkSolverPos> fksolver_;
                boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
                boost::shared_ptr<KDL::ChainIkSolverPos> ik_solver_;

                fksolver_.reset(new KDL::ChainFkSolverPos_recursive(AmigoConfig.getKinematicChain()));
                ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(AmigoConfig.getKinematicChain()));
                ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(AmigoConfig.getKinematicChain(), AmigoConfig.getJointMinima(), AmigoConfig.getJointMaxima(), *fksolver_, *ik_vel_solver_, 100) );

                KDL::JntArray q_out;

                /* check have we reached end of stem */
                if( (fabs(stem_intersection_xyz[2] - stemNodesZ.back()) < 0.05 && up > 0 ) || (fabs(stem_intersection_xyz[2] - stemNodesZ.front()) < 0.05 && up < 0) )
                    up = -up;

                KDL::Vector stem_inters( (double)stem_intersection_xyz[0], (double)stem_intersection_xyz[1], (double)stem_intersection_xyz[2]+0.001*(double) up );
                KDL::Frame f_in(AmigoStatus.getGripperKDLframe().M, stem_inters);

                int status = ik_solver_->CartToJnt(AmigoConfig.getJointSeeds(), f_in, q_out);

                if (status != 0 && DEBUG_IK)
                    INFO_STREAM("Inverse kinematics returns " << status );

                std::vector<std::string> joint_names = AmigoConfig.getJointNames();

                sensor_msgs::JointState arm_ref;
                arm_ref.header.stamp = ros::Time::now();
                arm_ref.position.clear();

                for(int i = 1; i<8; ++i)
                {
                    arm_ref.position.push_back(q_out(i));
                    arm_ref.name.push_back(joint_names[i]);
                }

                arm_reference_publisher.publish(arm_ref);

                sensor_msgs::JointState torso_ref;
                torso_ref.header.stamp = ros::Time::now();
                torso_ref.position.clear();
                torso_ref.position.push_back(q_out(0));
                torso_ref.name.push_back(joint_names[0]);

                torso_references_publisher.publish(torso_ref);

                //=========================================

            }

            if(!AmigoStatus.isUpToDate())
                INFO_STREAM("waiting for up to date robot status information");

            /* stop and publish sample timing, for profiling */
            sp.stopTimer("main");
            sp.publish();

        }

        else
            INFO_STREAM("error in loading config file!");

        /* wait for next sample */
        r.sleep();
        ros::spinOnce();

    }

    torso_measurements_subscriber.shutdown();
    arm_measurements_subscriber.shutdown();
    arm_reference_publisher.shutdown();
    visualization_publisher.shutdown();
    torso_references_publisher.shutdown();

    return 0;
};
