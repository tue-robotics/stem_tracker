#include "stem_tracker.h"

void showXYZInRviz(ros::Publisher* p_marker_pub, float x, float y, float z, float r, float g, float b, int id, const std::string ns){

    /* construct line strip marker object */
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/amigo/base_link";
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

    /* publish marker */
    p_marker_pub->publish( marker );

}

int main(int argc, char** argv){


    /* initialize node */
    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;
    ros::Rate r(UPDATE_RATE);

    tue::Configuration config;

    // Check if a config file was provided. If so, load it. If not, load a default config.
    if (argc >= 2)
    {
        std::string yaml_filename = argv[1];
        config.loadFromYAMLFile(yaml_filename);
    }
    else
    {
        // Get this nodes directory
        std::string ed_dir = ros::package::getPath("stem_tracker");

        // Load the YAML config file
        config.loadFromYAMLFile(ed_dir + "/config/config.yml");
    }

    config.value("root_link", ROOT_LINK);

    INFO_STREAM("\t\t\t Tha root link = " << ROOT_LINK);
    // ...


    if (config.hasError())
    {
        std::cout << "Could not load configuration: " << std::endl;
        std::cout << config.error() << std::endl;
        return 1;
    }

    /* initialize stem represenation object */

    RobotConfig AmigoConfig("amigo");
    StemRepresentation TomatoStem(1);

    TomatoStem.setRGB(STEM_R, STEM_G, STEM_B);
    TomatoStem.setThickness(STEM_THICKNESS);
    TomatoStem.addNodesXYZ(stemNodesX, stemNodesY, stemNodesZ);
    if(!USE_LEFTARM)
        TomatoStem.flipNodes();
    if(DEBUG)
        TomatoStem.printAll();

    /* initialize robot configuration object */

    if(USE_LEFTARM)
        AmigoConfig.setLeftArmIsPreferred();
    else
        AmigoConfig.setRightArmIsPreferred();

    AmigoConfig.loadUrdfFromRosparam(n, ROBOT_DESCRIPTION_ROSPARAM);
    AmigoConfig.loadKinematicTreeFromUrdf();

    if(USE_LEFTARM)
        AmigoConfig.loadKinematicChainFromTree(ROOT_LINK, LEFT_END_LINK);
    else
        AmigoConfig.loadKinematicChainFromTree(ROOT_LINK, RIGHT_END_LINK);

    AmigoConfig.loadJointLimits();

    if(DEBUG)
        AmigoConfig.printAll();

    /* initialize whisker interpretation object */

    WhiskerInterpreter TomatoWhiskerGripper(10, 1, 0.08, 0.2);

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

        /* publish linestrip marker to visualize stem */
        TomatoStem.showInRviz(&visualization_publisher);

        /* start sample timing, for profiling */
        sp.startTimer("main");

        /* check have we reached end of stem */
        state += up;
        if(state >= TomatoStem.getNumberOfNodes()-1 || state <= 0){
            up = -up;
            /* reached end of stem */
        }
        else{
            /* subgoal accomplished */
        }

        std::vector<float> gripper_xyz, stem_intersection_xyz;
        gripper_xyz = AmigoStatus.getGripperXYZ(AmigoConfig);
        if(AmigoStatus.isGripperXYZvalid())
            stem_intersection_xyz = TomatoStem.getStemXYZatZ(gripper_xyz[2]);

        if(AmigoStatus.isGripperXYZvalid() && TomatoStem.isXYZonStem(stem_intersection_xyz)){
            showXYZInRviz(&visualization_publisher, stem_intersection_xyz[0], stem_intersection_xyz[1], stem_intersection_xyz[2], 0.0f, 1.0f, 0.0f, 1, "stem_intersection");
            showXYZInRviz(&visualization_publisher, gripper_xyz[0], gripper_xyz[1], gripper_xyz[2], 1.0f, 0.0f, 0.0f, 2, "gripper_center");
        }

        TomatoWhiskerGripper.simulateWhiskerGripper(gripper_xyz, stem_intersection_xyz);
        TomatoWhiskerGripper.showForceInRviz(&visualization_publisher, gripper_xyz);

        /* stop and publish sample timing, for profiling */
        sp.stopTimer("main");
        sp.publish();

        /* selftrigger and wait for next sample */
        r.sleep();
        ros::spinOnce();

    }

    torso_measurements_subscriber.shutdown();
    arm_measurements_subscriber.shutdown();
    arm_reference_publisher.shutdown();
    visualization_publisher.shutdown();

    return 0;
};
