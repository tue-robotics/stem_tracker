#include "stem_tracker.h"

//TODO:
//- robot status ref geven naar robot config ipv friends
//- robot interface object
//- toevoegen kdl inverse kin



void showXYZInRviz(ros::Publisher* p_marker_pub, const std::string frame, float x, float y, float z, float r, float g, float b, int id, const std::string ns){

    visualization_msgs::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;

    marker.id = id;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    p_marker_pub->publish( marker );

}

void configure(tue::Configuration config){

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

    /* whisker configuration */
    config.value("n_whiskers", N_WHISKERS);
    config.value("whisker_length", WHISKER_LENGTH);
    config.value("gripper_diameter", GRIPPER_DIAMETER);

    /* tomato stem configuration */
    config.value("stem_thickness", STEM_THICKNESS);

    if (config.readGroup("stem_rgb")){
        config.value("r",STEM_RGB[0]);
        config.value("g",STEM_RGB[1]);
        config.value("b",STEM_RGB[2]);

        config.endGroup();
    }

    if (config.readArray("stem_nodes")){
        float tmp;
        stemNodesX.clear(); stemNodesY.clear(); stemNodesZ.clear();
        while(config.nextArrayItem()){
            config.value("x", tmp); stemNodesX.push_back(tmp);
            config.value("y", tmp); stemNodesY.push_back(tmp);
            config.value("z", tmp); stemNodesZ.push_back(tmp);
        }

        config.endArray();
    }

}

void initStem(StemRepresentation* stem){

    stem->setRGB(STEM_RGB[0], STEM_RGB[1], STEM_RGB[2]);
    stem->setThickness(STEM_THICKNESS);
    stem->loadNodesXYZ(stemNodesX, stemNodesY, stemNodesZ);
    stem->setFrame(BASE_FRAME);
    if(!USE_LEFTARM)
        stem->flipNodes();
    if(DEBUG)
        stem->printAll();
}

void initRobotConfig(RobotConfig* robot_config, ros::NodeHandle n){

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


int main(int argc, char** argv){

    bool initializing = true;
    int state = 0, up = 1;

    ros::Publisher visualization_publisher;
    ros::Publisher arm_reference_publisher;
    ros::Subscriber arm_measurements_subscriber;
    ros::Subscriber torso_measurements_subscriber;

    StatsPublisher sp;
    tue::Configuration config;

    /* load yaml config file */

    if (argc >= 2){
        std::string yaml_filename = argv[1];
        config.loadFromYAMLFile(yaml_filename);
    } else {
        std::string this_package_dir = ros::package::getPath(THIS_PACKAGE);
        config.loadFromYAMLFile(this_package_dir + "/config/config.yml");
    }

    configure(config);

    if (config.hasError()){
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

    RobotStatus AmigoStatus(AmigoConfig.getKinematicChain().getNrOfJoints());

    /* initialize node communication */

    visualization_publisher = n.advertise<visualization_msgs::Marker>( "visualization_marker", 100 );
    //      marker id   0   stem
    //      marker id   1   stem-gripper intersection
    //      marker id   2   gripper center
    //      marker id   3   whisker force

    if (USE_LEFTARM)
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 0);
    else
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 0);

    torso_measurements_subscriber = n.subscribe("/amigo/torso/measurements", 1000, &RobotStatus::receivedTorsoMsg, &AmigoStatus);

    if (USE_LEFTARM)
        arm_measurements_subscriber = n.subscribe("/amigo/left_arm/measurements", 1000, &RobotStatus::receivedArmMsg, &AmigoStatus);
    else
        arm_measurements_subscriber = n.subscribe("/amigo/right_arm/measurements", 1000, &RobotStatus::receivedArmMsg, &AmigoStatus);

    /* initialize profiling */
    sp.initialize();

    if (initializing){
        /* bring arm to initial position */
        arm_reference_publisher.publish(AmigoConfig.getInitialPoseMsg());
        INFO_STREAM("Can I continue? Press enter");
        std::cin.get();
        initializing = false;
    }

    /* update loop */
    while(ros::ok()){

        if (config.sync()){
            /* Configuration changed, so reconfigure */
            configure(config);

            initStem(&TomatoStem);
            initRobotConfig(&AmigoConfig, n);
        }

        if (!config.hasError()){

            /* publish linestrip marker to visualize stem */
            TomatoStem.showInRviz(&visualization_publisher, "stem");

            /* start sample timing, for profiling */
            sp.startTimer("main");

            /* check have we reached end of stem */
            state += up;
            if(state >= TomatoStem.getNumberOfNodes()-1 || state <= 0){
                /* reached end of stem */
                up = -up;
            }

            std::vector<float> gripper_xyz, stem_intersection_xyz;
            gripper_xyz = AmigoStatus.getGripperXYZ(AmigoConfig);
            if(AmigoStatus.isGripperXYZvalid())
                stem_intersection_xyz = TomatoStem.getStemXYZatZ(gripper_xyz[2]);

            if(AmigoStatus.isGripperXYZvalid() && TomatoStem.isXYZonStem(stem_intersection_xyz)){
                showXYZInRviz(&visualization_publisher, BASE_FRAME, stem_intersection_xyz[0], stem_intersection_xyz[1], stem_intersection_xyz[2], 0.0f, 1.0f, 0.0f, 1, "stem_intersection");
                showXYZInRviz(&visualization_publisher, BASE_FRAME, gripper_xyz[0], gripper_xyz[1], gripper_xyz[2], 1.0f, 0.0f, 0.0f, 2, "gripper_center");
                TomatoWhiskerGripper.simulateWhiskerGripper(gripper_xyz, stem_intersection_xyz);
                TomatoWhiskerGripper.showForceInRviz(&visualization_publisher, gripper_xyz);
            }

            /* stop and publish sample timing, for profiling */
            sp.stopTimer("main");
            sp.publish();
        }

        /* wait for next sample */
        r.sleep();
        ros::spinOnce();
    }

    torso_measurements_subscriber.shutdown();
    arm_measurements_subscriber.shutdown();
    arm_reference_publisher.shutdown();
    visualization_publisher.shutdown();

    return 0;
};
