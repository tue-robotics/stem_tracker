#include "configurer.h"

#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "robotinterface.h"
#include "whiskerinterpreter.h"
#include "stemtrackcontroller.h"
#include "stemtrackmonitor.h"
#include "visualizationinterface.h"


#include "ros/ros.h"  /* Only for ROS_INFO_STREAM */

const double Configurer::extractDouble(const tue::Configuration& config, const std::string& name)
{
    double tmp;
    config.value(name, tmp);
    return tmp;
}

const float Configurer::extractFloat(const tue::Configuration& config, const std::string& name)
{
    float tmp;
    config.value(name, tmp);
    return tmp;
}

const int Configurer::extractInt(const tue::Configuration& config, const std::string& name)
{
    int tmp;
    config.value(name, tmp);
    return tmp;
}

const bool Configurer::extractBool(const tue::Configuration& config, const std::string& name)
{
    bool tmp;
    config.value(name, tmp);
    return tmp;
}

const std::string Configurer::extractString(const tue::Configuration& config, const std::string& name)
{
    std::string tmp;
    config.value(name, tmp);
    return tmp;
}

const std::string Configurer::getBaseFrame(const tue::Configuration& config)
{
    return extractString(config, "base_frame");
}

const int Configurer::getUpdateRate(const tue::Configuration& config)
{
    return extractInt(config, "update_rate");
}

const int Configurer::getLoglevel(const tue::Configuration& config)
{
    return extractInt(config,"loglevel");
}

const bool Configurer::getUseLeft(const tue::Configuration& config)
{
    return extractBool(config, "use_leftarm");
}

void Configurer::configureStemRepresentation(tue::Configuration& config, StemRepresentation& stem_representation)
{
    stem_representation.setLinTangentDistance( extractFloat(config, "lin_tan_d") );

    std::vector<float> stemNodesX, stemNodesY, stemNodesZ;

    if (config.readArray("stem_nodes"))
    {
        while(config.nextArrayItem())
        {
            stemNodesX.push_back( extractFloat(config, "x") );
            stemNodesY.push_back( extractFloat(config, "y") );
            stemNodesZ.push_back( extractFloat(config, "z") );
        }

        config.endArray();
    }

    stem_representation.loadNodesXYZ( stemNodesX, stemNodesY, stemNodesZ);

    if( !getUseLeft(config) )
        stem_representation.flipNodes();

    if( getLoglevel(config) > 0 )
    {
        ROS_INFO_STREAM("===================================================");
        ROS_INFO_STREAM("Configured stem representation object for stem " << stem_representation.getStemID() );
        stem_representation.printAll();
    }
}

void Configurer::configureWhiskerInterpreter(const tue::Configuration& config, WhiskerInterpreter& whisker_interpreter)
{
    whisker_interpreter.setNumberOfWhiskers( extractInt(config, "n_whiskers") );
    whisker_interpreter.setWhiskerLength( extractFloat(config, "whisker_length") );
    whisker_interpreter.setGripperDiameter( extractFloat(config, "gripper_diameter") );
    whisker_interpreter.setMaxWhiskerForce( extractFloat(config, "max_whisker_force") );

    if( getLoglevel(config) > 0 )
    {
        ROS_INFO_STREAM("=====================================================");
        ROS_INFO_STREAM("Configured whisker interpreter object for gripper " << whisker_interpreter.getGripperID() );
    }
}

void Configurer::configureRobotRepresentation(tue::Configuration& config, RobotRepresentation& robot_representation)
{
    if( getUseLeft(config) )
        robot_representation.setLeftArmIsPreferred();
    else
        robot_representation.setRightArmIsPreferred();

    robot_representation.loadUrdfFromFile( extractString(config, "robot_urdf_file") );
    robot_representation.loadKinematicTreeFromUrdf();

    if( getUseLeft(config) )
        robot_representation.loadKinematicChainFromTree( extractString(config, "root_link"), extractString(config, "left_end_link") );
    else
        robot_representation.loadKinematicChainFromTree( extractString(config, "root_link"), extractString(config, "right_end_link") );

    robot_representation.loadJointLimits();

    std::vector<float> tmp;
    if (config.readArray("initial_pose"))
    {
        while(config.nextArrayItem())
            tmp.push_back( extractFloat(config, "q") );

        config.endArray();
    }
    robot_representation.setInitialPoseJointRefs( tmp );

    if( getLoglevel(config) > 0 )
    {
        ROS_INFO_STREAM("====================================================");
        ROS_INFO_STREAM("Configured robot representation object of robot " << robot_representation.getName().c_str() );
        robot_representation.printAll();
    }
}

void Configurer::configureRobotStatus(const tue::Configuration& config, RobotStatus& robot_status)
{
    robot_status.setXYZreachedThreshold( extractDouble(config, "xyz_reached_threshold") );
    robot_status.setUpToDateThreshold( extractDouble(config, "up_to_date_threshold") );
    robot_status.setPosReachedThreshold( extractFloat(config, "pos_reached_threshold") );

    if( getLoglevel(config) > 0 )
    {
        ROS_INFO_STREAM("=====================================");
        ROS_INFO_STREAM("Configured robot status object");
    }
}

void Configurer::configureStemTrackController(const tue::Configuration& config, StemTrackController& stem_track_controller)
{
    stem_track_controller.setMaxZvelocity( extractFloat(config, "max_z_dot") );
    stem_track_controller.setUpdateRate( extractInt(config, "update_rate") );
    stem_track_controller.setTiltWithStem( extractBool(config, "tilt_with_stem") );
    stem_track_controller.setDebugIKsolver( extractBool(config, "debug_ik_solver") );
    stem_track_controller.setUseInverseVelocitySolverOnly( extractBool(config, "ik_vel_only") );

    if( getLoglevel(config) > 0 )
    {
        ROS_INFO_STREAM("=============================================");
        ROS_INFO_STREAM("Configured stem track controller object");
    }
}

void Configurer::configureRobotInterface(const tue::Configuration& config, RobotInterface& robot_interface)
{
    robot_interface.connectToAmigoArm( getUseLeft(config) );
    robot_interface.connectToAmigoTorso();

    if( getLoglevel(config) > 0 )
    {
        ROS_INFO_STREAM("=============================================");
        ROS_INFO_STREAM("Configured robot interface object");
    }
}

void Configurer::configureStemTrackMonitor(const tue::Configuration& config, StemTrackMonitor& stemtrack_monitor)
{
    stemtrack_monitor.setDebugStateParameter( extractBool(config, "debug_state_par") );

    if( getLoglevel(config) > 0 )
    {
        ROS_INFO_STREAM("=============================================");
        ROS_INFO_STREAM("Configured stemtrack monitor object");
    }
}

void Configurer::configureVisualizationInterface(const tue::Configuration& config, VisualizationInterface& visualization_interface)
{
    visualization_interface.connectToRos( extractString(config, "topic_name"), extractInt(config, "buffer_size") );

    if( getLoglevel(config) > 0 )
    {
        ROS_INFO_STREAM("=============================================");
        ROS_INFO_STREAM("Configured visualization object");
        ROS_INFO_STREAM("=============================================");

    }
}

Configurer::~Configurer()
{
    //destructor
}
