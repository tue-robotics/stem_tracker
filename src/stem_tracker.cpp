#define DEBUG                           true                            // if true additional information will be printed
#define INFO_STREAM                     ROS_INFO_STREAM

#include "stem_tracker.hpp"

// Loading configuration
#include <tue/config/configuration.h>
#include <ros/package.h>


#define UPDATE_RATE                     2                               // spin rate of this node, in hz

#define USE_LEFTARM                     true                            // use left arm if true, else use right arm
#define ROBOT_DESCRIPTION_ROSPARAM      "/amigo/robot_description"      // amigo urdf model gets loaded in this rosparam

#define STEM_R                          0.05
#define STEM_G                          0.65
#define STEM_B                          0.35
#define STEM_THICKNESS                  0.03                            // thickness in meters

float X[] = {  0.3,     0.35,   0.3,    0.35,   0.3,    0.25    };
float Y[] = {  0.3,     0.35,   0.35,   0.4,    0.55,   0.6     };
float Z[] = {  0.4,     0.6,    0.9,    1.2,    1.4,    1.6     };
std::vector<float> stemNodesX(X, X + sizeof(X) / sizeof(*X) );
std::vector<float> stemNodesY(Y, Y + sizeof(Y) / sizeof(*Y) );
std::vector<float> stemNodesZ(Z, Z + sizeof(Z) / sizeof(*Z) );

std::string ROOT_LINK;

#define LEFT_END_LINK   "grippoint_left"
#define RIGHT_END_LINK  "grippoint_right"


\
/* initialize */

bool initializing = true;
int state = 0;
int i, up = 1;
ros::Publisher visualization_publisher;
ros::Publisher arm_reference_publisher;
ros::Subscriber arm_measurements_subscriber;
ros::Subscriber torso_measurements_subscriber;

StatsPublisher sp;


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

    visualization_publisher = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );

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

        /* stop and publish sample timing, for profiling */
        sp.stopTimer("main");
        sp.publish();

        /* selftrigger and wait for next sample */
        r.sleep();
        ros::spinOnce();

        AmigoStatus.printAll();

        //    ===============================

        KDL::ChainFkSolverPos_recursive forward_kinematics_solver = KDL::ChainFkSolverPos_recursive(AmigoConfig.getKinematicChain());
        bool kin_stat;
        KDL::Frame cartpos;
        kin_stat = forward_kinematics_solver.JntToCart(AmigoStatus.getJointStatus(),cartpos);
        INFO_STREAM("x = " << cartpos.p.x() << " y = " << cartpos.p.y() << " z = " << cartpos.p.z());

        std::vector<float> gripper_center, stem_center, whisker_force;
        gripper_center.push_back(cartpos.p.x());
        gripper_center.push_back(cartpos.p.y());
        gripper_center.push_back(cartpos.p.z());

        stem_center = TomatoStem.getXYatZ(cartpos.p.z());
        TomatoWhiskerGripper.simulateWhiskerGripper(gripper_center, stem_center);
        TomatoWhiskerGripper.showForceInRviz(&visualization_publisher, gripper_center);
    }

    torso_measurements_subscriber.shutdown();
    arm_measurements_subscriber.shutdown();
    arm_reference_publisher.shutdown();
    visualization_publisher.shutdown();

    return 0;
};
