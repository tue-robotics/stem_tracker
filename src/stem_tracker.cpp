#include "stem_tracker.hpp"

/* configure */

#define USE_LEFTARM                     true                            // use left arm if true, else use right arm
#define ROBOT_DESCRIPTION_ROSPARAM      "/amigo/robot_description"      // amigo urdf model gets loaded in this rosparam
#define DEBUG                           true                            // if true additional information will be printed
#define UPDATE_RATE                     2                               // spin rate of this node, in hz
#define STEM_R                          0.05
#define STEM_G                          0.65
#define STEM_B                          0.35

float stemNodesXYZ[] = { 0.3, 0.3, 0.4,     // nodes of a virtual stem,
                         0.35, 0.35, 0.6,   // list of coordinates xyzxyzxyz...
                         0.3, 0.35, 0.9,    // defined in amigo base_link
                         0.35, 0.4, 1.2,    // y-coordinates will be
                         0.3, 0.55, 1.4,    // flipped if use_leftarm is false
                         0.25, 0.6, 1.6};

#define FLOATS_PER_NODE                 3

\
/* initialize */

int state = 0;
int i, up = 1;
ros::Publisher visualization_publisher;
ros::Publisher arm_reference_publisher;
sensor_msgs::JointState arm_joint_msg;
StatsPublisher sp;

RobotConfig AmigoConfig("amigo");
VirtualStem TomatoStem(1);

int main(int argc, char** argv){


    /* initialize node */
    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;
    ros::Rate r(UPDATE_RATE);

    /* initialize node communication */
    visualization_publisher = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    if (USE_LEFTARM)
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 0);
    else
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 0);

    /* create a vitual stem */
    TomatoStem.setFloatsPerNode(FLOATS_PER_NODE);
    TomatoStem.setRGB(STEM_R, STEM_G, STEM_B);
    TomatoStem.addNodes(stemNodesXYZ, sizeof(stemNodesXYZ)/sizeof(*stemNodesXYZ)/3);
    if(!USE_LEFTARM)
        TomatoStem.flipNodes();
    if(DEBUG)
        TomatoStem.printAll();

    /* load robot hardware configuration */
    AmigoConfig.loadUrdfFromRosparam(n, ROBOT_DESCRIPTION_ROSPARAM);
    AmigoConfig.loadKinematicTreeFromUrdf();
    AmigoConfig.setLeftArmIsPreferred(USE_LEFTARM);

    if(DEBUG)
        AmigoConfig.printAll();


    /* initialize profiling */
    sp.initialize();

    /* update loop */
    while(ros::ok()){

        /* start sample timing, for profiling */
        sp.startTimer("main");

        /* publish linestrip marker to visualize stem */
        visualizeStem(visualization_publisher, stemNodesXYZ, sizeof(stemNodesXYZ)/sizeof(*stemNodesXYZ)/3);

        /* bring arm to initial position */
        arm_joint_msg = amigoGetInitialPosition(USE_LEFTARM);
        arm_reference_publisher.publish(arm_joint_msg);

        /* check have we reached end of stem */
        state += up;
        if(state>=(int)sizeof(stemNodesXYZ)/sizeof(double_t)/3-1 || state < 0){
            up = -up;
            ROS_INFO("reached end of stem");
            if(state<0){
                state = 0;
            }
        }
        else{
            ROS_INFO("subgoal accomplished");
        }

        /* stop and publish sample timing, for profiling */
        sp.stopTimer("main");
        sp.publish();

        /* selftrigger and wait for next sample */
        r.sleep();
        ros::spinOnce();

    }

    return 0;
};


