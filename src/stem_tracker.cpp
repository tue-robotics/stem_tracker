#define DEBUG                           true                            // if true additional information will be printed
#define INFO_STREAM                     ROS_INFO_STREAM

#include "stem_tracker.hpp"


#define UPDATE_RATE                     2                               // spin rate of this node, in hz

#define USE_LEFTARM                     true                            // use left arm if true, else use right arm
#define ROBOT_DESCRIPTION_ROSPARAM      "/amigo/robot_description"      // amigo urdf model gets loaded in this rosparam

#define STEM_R                          0.05
#define STEM_G                          0.65
#define STEM_B                          0.35
#define STEM_THICKNESS                  0.03                            // thickness in meters

float stemNodesXYZ[] = { 0.3, 0.3, 0.4,     // nodes of a virtual stem,
                         0.35, 0.35, 0.6,   // list of coordinates xyzxyzxyz...
                         0.3, 0.35, 0.9,    // defined in amigo base_link
                         0.35, 0.4, 1.2,    // y-coordinates will be
                         0.3, 0.55, 1.4,    // flipped if use_leftarm is false
                         0.25, 0.6, 1.6};

#define NODE_DIMENSION                 3

#define ROOT_LINK       "base_link"
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

    /* initialize stem represenation object */

    RobotConfig AmigoConfig("amigo");
    StemRepresentation TomatoStem(1);

    TomatoStem.setNodeDimension(NODE_DIMENSION);
    TomatoStem.setRGB(STEM_R, STEM_G, STEM_B);
    TomatoStem.setThickness(STEM_THICKNESS);
    TomatoStem.addNodes(stemNodesXYZ, sizeof(stemNodesXYZ)/sizeof(*stemNodesXYZ)/3);
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

//        INFO_STREAM("state = " << state);

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

            // Create solver based on kinematic chain
//                KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(AmigoConfig.getKinematicChain());

//                // Create joint array
//                unsigned int nj = AmigoConfig.getKinematicChain().getNrOfJoints();
//                KDL::JntArray jointpositions = KDL::JntArray(nj);

                // Assign some values to the joint positions
//                for(unsigned int i=0;i<nj;i++){

//                    jointpositions(i)=0.0;
//                }

//                // Create the frame that will contain the results
//                KDL::Frame cartpos;

//                // Calculate forward position kinematics
//                bool kinematics_status;
//                kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
//                if(kinematics_status>=0){
//        //            INFO_STREAM(cartpos);
//                    INFO_STREAM("Succes, thanks KDL!");
//                }else{
//                    INFO_STREAM("Error: could not calculate forward kinematics :(");
//                }

        //        ===============================

        KDL::ChainFkSolverPos_recursive forward_kinematics_solver = KDL::ChainFkSolverPos_recursive(AmigoConfig.getKinematicChain());
        bool kin_stat;
        KDL::Frame cartpos;
        kin_stat = forward_kinematics_solver.JntToCart(AmigoStatus.getJointStatus(),cartpos);
        INFO_STREAM("x = " << cartpos.p.x() << " y = " << cartpos.p.y() << " z = " << cartpos.p.z());
    }

    torso_measurements_subscriber.shutdown();
    arm_measurements_subscriber.shutdown();
    arm_reference_publisher.shutdown();
    visualization_publisher.shutdown();

    return 0;
};
