#include "stem_tracker.h"

/* configure */

#define USE_LEFTARM                     true                            // use left arm if true, else use right arm
#define ROBOT_DESCRIPTION_ROSPARAM      "/amigo/robot_description"      // xml, generated from urdf model


double stemNodesXYZ[] = {0.3, 0.3, 0.4,     // nodes of a virtual stem,
                         0.35, 0.35, 0.6,   // list of coordinates xyzxyzxyz...
                         0.3, 0.35, 0.9,    // defined in amigo base_link
                         0.35, 0.4, 1.2,    // for left-arm, y-coordinates will be
                         0.3, 0.55, 1.4,    // flipped if use_leftarm is false
                         0.25, 0.6, 1.6};
\
/* initialize */

int state = 0;
int i, up = 1;
ros::Publisher visualization_publisher;
ros::Publisher arm_reference_publisher;
sensor_msgs::JointState arm_joint_msg;
std::string robot_description_xml;
StatsPublisher sp;
Tree kinematic_tree;


int main(int argc, char** argv){


    /* initialize node */
    ros::init(argc, argv, "stem_tracker");
    ros::NodeHandle n;
    ros::Rate r(2); // hz

    /* initialize node communication */
    visualization_publisher = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    if (USE_LEFTARM)
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 0);
    else
        arm_reference_publisher = n.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 0);

    /* flip stem coordinates if use right instead of left arm */
    if (!USE_LEFTARM){
        for(i=1;i<(int)sizeof(stemNodesXYZ)/sizeof(double_t);i+=3)
            stemNodesXYZ[i] *= -1;
    }

    /* get robot description from ros parameter server */
    robot_description_xml = getRobotDescription(n, ROBOT_DESCRIPTION_ROSPARAM);

    /* turn xml robot description in kdl tree */
    kinematic_tree = getKinematicTree(robot_description_xml);


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


