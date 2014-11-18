#include "configurer.h"

void Configurer::loadGlobalConfig(tue::Configuration config)
{

    INFO_STREAM("=========================================");
    INFO_STREAM("Configuring global config parameters");

    /* general configuration */
    config.value("base_frame", g_BASE_FRAME);
    INFO_STREAM("base_frame = " << g_BASE_FRAME);
    config.value("debug", g_DEBUG);
    INFO_STREAM("debug = " << g_DEBUG);
    config.value("update_rate", g_UPDATE_RATE);
    INFO_STREAM("update_rate = " << g_UPDATE_RATE);

}

std::string Configurer::getBaseFrame(tue::Configuration config)
{
    config.value("base_frame", g_BASE_FRAME);
    return g_BASE_FRAME;
}

void Configurer::configureStemRepresentation(tue::Configuration config, StemRepresentation* p_stem_representation)
{

    INFO_STREAM("===================================================");
    INFO_STREAM("Configuring stem representation object for stem " << p_stem_representation->getStemID() );

    config.value("lin_tan_d", sr_LIN_TAN_D);
    p_stem_representation->setLinTangentDistance(sr_LIN_TAN_D);

    /* tomato stem configuration */
    if (config.readArray("stem_nodes"))
    {
        float tmp;
        sr_stemNodesX.clear(); sr_stemNodesY.clear(); sr_stemNodesZ.clear();

        while(config.nextArrayItem())
        {
            config.value("x", tmp); sr_stemNodesX.push_back(tmp);
            config.value("y", tmp); sr_stemNodesY.push_back(tmp);
            config.value("z", tmp); sr_stemNodesZ.push_back(tmp);
        }

        config.endArray();
    }

    p_stem_representation->loadNodesXYZ(sr_stemNodesX, sr_stemNodesY, sr_stemNodesZ);

    config.value("use_leftarm", rr_USE_LEFTARM);

    if(!rr_USE_LEFTARM)
        p_stem_representation->flipNodes();

    config.value("debug", g_DEBUG);

    if(g_DEBUG)
        p_stem_representation->printAll();
}

int Configurer::getUpdateRate(tue::Configuration config)
{
    config.value("update_rate", g_UPDATE_RATE);
    return g_UPDATE_RATE;
}

void Configurer::configureWhiskerInterpreter(tue::Configuration config, WhiskerInterpreter* p_whisker_interpreter)
{
    INFO_STREAM("=====================================================");
    INFO_STREAM("Configuring whisker interpreter object for gripper " << p_whisker_interpreter->getGripperID() );

    config.value("n_whiskers", wi_N_WHISKERS);
    config.value("whisker_length", wi_WHISKER_LENGTH);
    config.value("gripper_diameter", wi_GRIPPER_DIAMETER);
    config.value("max_whisker_force", wi_MAX_WHISKER_FORCE);

    p_whisker_interpreter->setGripperDiameter(wi_GRIPPER_DIAMETER);
    p_whisker_interpreter->setNumberOfWhiskers(wi_N_WHISKERS);
    p_whisker_interpreter->setWhiskerLength(wi_WHISKER_LENGTH);
    p_whisker_interpreter->setMaxWhiskerForce(wi_MAX_WHISKER_FORCE);

}

void Configurer::configureRobotRepresentation(tue::Configuration config, RobotRepresentation* p_robot_representation, ros::NodeHandle n)
{

    INFO_STREAM("====================================================");
    INFO_STREAM("Configuring robot representation object of robot " << p_robot_representation->getName().c_str() );

    config.value("root_link", rr_ROOT_LINK);
    config.value("use_leftarm", rr_USE_LEFTARM);
    config.value("left_end_link", rr_LEFT_END_LINK);
    config.value("right_end_link", rr_RIGHT_END_LINK);

    if(rr_USE_LEFTARM)
        p_robot_representation->setLeftArmIsPreferred();
    else
        p_robot_representation->setRightArmIsPreferred();

    config.value("robot_urdf_file", rr_URDF_FILENAME);
    p_robot_representation->loadUrdfFromFile( rr_URDF_FILENAME);
    p_robot_representation->loadKinematicTreeFromUrdf();

    if(rr_USE_LEFTARM)
        p_robot_representation->loadKinematicChainFromTree(rr_ROOT_LINK, rr_LEFT_END_LINK);
    else
        p_robot_representation->loadKinematicChainFromTree(rr_ROOT_LINK, rr_RIGHT_END_LINK);

    p_robot_representation->loadJointLimits();

    if (config.readArray("initial_pose"))
    {
        float tmp;
        rr_INITIAL_POSE.clear();

        while(config.nextArrayItem())
        {
            config.value("q", tmp); rr_INITIAL_POSE.push_back(tmp);
        }

        config.endArray();
    }
    p_robot_representation->setInitialPoseJointRefs(rr_INITIAL_POSE);

    config.value("debug", g_DEBUG);
    if(g_DEBUG)
        p_robot_representation->printAll();
}

void Configurer::configureRobotStatus(tue::Configuration config, RobotStatus* p_robot_status)
{
    INFO_STREAM("=====================================");
    INFO_STREAM("Configuring robot status object");

    config.value("up_to_date_threshold", rs_UP_TO_DATE_THRESHOLD);
    INFO_STREAM("up_to_date_threshold = " << rs_UP_TO_DATE_THRESHOLD);

    config.value("pos_reached_threshold", rs_POS_REACHED_THRESHOLD);
    INFO_STREAM("pos_reached_threshold = " << rs_POS_REACHED_THRESHOLD);

    p_robot_status->setPosReachedThreshold(rs_POS_REACHED_THRESHOLD);
    p_robot_status->setUpToDateThreshold( rs_UP_TO_DATE_THRESHOLD );
}

void Configurer::configureStemTrackController(tue::Configuration config, StemTrackController* p_stem_track_controller)
{
    INFO_STREAM("=============================================");
    INFO_STREAM("Configuring stem track controller object");

    config.value("max_z_dot", stc_MAX_Z_DOT);
    p_stem_track_controller->setMaxZvelocity(stc_MAX_Z_DOT);

    config.value("update_rate", g_UPDATE_RATE);
    p_stem_track_controller->setUpdateRate(g_UPDATE_RATE);

    config.value("tilt_with_stem", stc_TILT_WITH_STEM);
    p_stem_track_controller->setTiltWithStem(stc_TILT_WITH_STEM);

    config.value("debug_ik_solver", stc_DEBUG_IK_SOLVER);
    p_stem_track_controller->setDebugIKsolver(stc_DEBUG_IK_SOLVER);
}

void Configurer::configureRobotInterface(tue::Configuration config, RobotInterface* p_robot_interface)
{
    INFO_STREAM("=============================================");
    INFO_STREAM("Configuring robot interface object");

    config.value("use_leftarm", rr_USE_LEFTARM);

    p_robot_interface->connectToAmigoArm(rr_USE_LEFTARM);
    p_robot_interface->connectToAmigoTorso();

}

void Configurer::configureStemTrackMonitor(tue::Configuration config, StemTrackMonitor* p_stemtrack_monitor)
{
    INFO_STREAM("=============================================");

    INFO_STREAM("Configuring stemtrack monitor object");

    config.value("debug_state_par", stm_DEBUG_STATE_PAR);

    p_stemtrack_monitor->setDebugStateParameter(stm_DEBUG_STATE_PAR);

    INFO_STREAM("=============================================");
}

Configurer::~Configurer()
{
    //destructor
}
