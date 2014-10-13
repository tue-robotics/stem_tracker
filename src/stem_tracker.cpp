#define DEBUG                           true                            // if true additional information will be printed
#define INFO_STREAM                     ROS_INFO_STREAM

#include "stem_tracker.hpp"


#define UPDATE_RATE                     2                               // spin rate of this node, in hz

#define USE_LEFTARM                     true                            // use left arm if true, else use right arm
#define ROBOT_DESCRIPTION_ROSPARAM      "/amigo/robot_description"      // amigo urdf model gets loaded in this rosparam

#define STEM_R                          0.05
#define STEM_G                          0.65
#define STEM_B                          0.35
#define STEM_THICKNESS                  0.03                            // thickness in cm

float stemNodesXYZ[] = { 0.3, 0.3, 0.4,     // nodes of a virtual stem,
                         0.35, 0.35, 0.6,   // list of coordinates xyzxyzxyz...
                         0.3, 0.35, 0.9,    // defined in amigo base_link
                         0.35, 0.4, 1.2,    // y-coordinates will be
                         0.3, 0.55, 1.4,    // flipped if use_leftarm is false
                         0.25, 0.6, 1.6};

#define NODE_DIMENSION                 3

\
/* initialize */

int state = 0;
int i, up = 1;
ros::Publisher visualization_publisher;
ros::Publisher arm_reference_publisher;
StatsPublisher sp;

RobotConfig AmigoConfig("amigo");
VirtualStem TomatoStem(1);


int main(int argc, char** argv){


    /* initialize node */
    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;
    ros::Rate r(UPDATE_RATE);

    /* initialize node communication */
    visualization_publisher = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
    if (USE_LEFTARM)
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 0);
    else
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 0);

    /* create a vitual stem */
    TomatoStem.setFloatsPerNode(NODE_DIMENSION);
    TomatoStem.setRGB(STEM_R, STEM_G, STEM_B);
    TomatoStem.setThickness(STEM_THICKNESS);
    TomatoStem.addNodes(stemNodesXYZ, sizeof(stemNodesXYZ)/sizeof(*stemNodesXYZ)/3);
    if(!USE_LEFTARM)
        TomatoStem.flipNodes();
    if(DEBUG)
        TomatoStem.printAll();

    /* load robot hardware configuration */
    AmigoConfig.loadUrdfFromRosparam(n, ROBOT_DESCRIPTION_ROSPARAM);
    AmigoConfig.loadKinematicTreeFromUrdf();
    if(USE_LEFTARM)
        AmigoConfig.setLeftArmIsPreferred();
    else
        AmigoConfig.setRightArmIsPreferred();

    if(DEBUG)
        AmigoConfig.printAll();

    /* initialize profiling */
    sp.initialize();


    /* update loop */
    while(ros::ok()){

        /* publish linestrip marker to visualize stem */
        TomatoStem.showInRviz(&visualization_publisher);

        /* bring arm to initial position */
        AmigoConfig.publishInitialPose(&arm_reference_publisher);

        /* start sample timing, for profiling */
        sp.startTimer("main");

        /* check have we reached end of stem */
        state += up;
        if(state>=(int)sizeof(stemNodesXYZ)/sizeof(*stemNodesXYZ)/NODE_DIMENSION-1 || state < 0){
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


