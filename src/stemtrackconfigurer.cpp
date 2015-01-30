#include "stemtrackconfigurer.h"

#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "robotinterface.h"
#include "whiskerinterpreter.h"
#include "stemtrackcontroller.h"
#include "stemtrackmonitor.h"
#include "visualizationinterface.h"
#include "loggingmacros.h"

template <class T>
const T StemTrackConfigurer::getConfigPar(tue::Configuration& config, const std::string& name)
{
    T tmp;
    config.value(name, tmp);
    return tmp;
}

const std::string StemTrackConfigurer::getBaseFrame(tue::Configuration& config)
{
    return getConfigPar<std::string>(config, "base_frame");
}

const int StemTrackConfigurer::getUpdateRate(tue::Configuration& config)
{
    return getConfigPar<int>(config, "update_rate");
}

const int StemTrackConfigurer::getLoglevel(tue::Configuration& config)
{
    return getConfigPar<int>(config,"loglevel");
}

const bool StemTrackConfigurer::getUseLeft(tue::Configuration& config)
{
    return getConfigPar<bool>(config, "use_leftarm");
}

void StemTrackConfigurer::configureStemRepresentation(tue::Configuration& config, StemRepresentation& stem_representation)
{
    stem_representation.setLinTangentDistance( getConfigPar<float>(config, "lin_tan_d") );

    std::vector<float> stemNodesX, stemNodesY, stemNodesZ;

    if (config.readArray("stem_nodes"))
    {
        while(config.nextArrayItem())
        {
            stemNodesX.push_back( getConfigPar<float>(config, "x") );
            stemNodesY.push_back( getConfigPar<float>(config, "y") );
            stemNodesZ.push_back( getConfigPar<float>(config, "z") );
        }

        config.endArray();
    }

    stem_representation.loadNodesXYZ( stemNodesX, stemNodesY, stemNodesZ);

    if( !getUseLeft(config) )
        stem_representation.flipNodes();

    if( getLoglevel(config) > 0 )
    {
        INFO_STREAM("===================================================");
        INFO_STREAM("Configured stem representation object for stem " << stem_representation.getStemID() );
        stem_representation.printAll();
    }
}

void StemTrackConfigurer::configureWhiskerInterpreter(tue::Configuration& config, WhiskerInterpreter& whisker_interpreter)
{
    whisker_interpreter.setNumberOfWhiskers( getConfigPar<int>(config, "n_whiskers") );
    whisker_interpreter.setWhiskerLength( getConfigPar<float>(config, "whisker_length") );
    whisker_interpreter.setGripperDiameter( getConfigPar<float>(config, "gripper_diameter") );
    whisker_interpreter.setMaxWhiskerForce( getConfigPar<float>(config, "max_whisker_force") );

    if( getLoglevel(config) > 0 )
    {
        INFO_STREAM("=====================================================");
        INFO_STREAM("Configured whisker interpreter object" );
    }
}

void StemTrackConfigurer::configureRobotRepresentation(tue::Configuration& config, RobotRepresentation& robot_representation)
{
    if( getUseLeft(config) )
        robot_representation.setLeftArmIsPreferred();
    else
        robot_representation.setRightArmIsPreferred();

    robot_representation.loadUrdfFromFile( getConfigPar<std::string>(config, "robot_urdf_file") );
    robot_representation.loadKinematicTreeFromUrdf();

    if( getUseLeft(config) )
        robot_representation.loadKinematicChainFromTree( getConfigPar<std::string>(config, "root_link"), getConfigPar<std::string>(config, "left_end_link") );
    else
        robot_representation.loadKinematicChainFromTree( getConfigPar<std::string>(config, "root_link"), getConfigPar<std::string>(config, "right_end_link") );

    robot_representation.loadJointLimits();

    std::vector<float> tmp;
    if (config.readArray("initial_pose"))
    {
        while(config.nextArrayItem())
            tmp.push_back( getConfigPar<float>(config, "q") );

        config.endArray();
    }
    robot_representation.setInitialPoseJointRefs( tmp );

    if( getLoglevel(config) > 0 )
    {
        INFO_STREAM("====================================================");
        INFO_STREAM("Configured robot representation object of robot " << robot_representation.getName().c_str() );
        robot_representation.printAll();
    }
}

void StemTrackConfigurer::configureRobotStatus(tue::Configuration& config, RobotStatus& robot_status)
{
    robot_status.setXYZreachedThreshold( getConfigPar<double>(config, "xyz_reached_threshold") );
    robot_status.setUpToDateThreshold( getConfigPar<double>(config, "up_to_date_threshold") );
    robot_status.setPosReachedThreshold( getConfigPar<double>(config, "pos_reached_threshold") );
    robot_status.setNumberOfWhiskers( getConfigPar<int>(config, "n_whiskers") );

    if( getLoglevel(config) > 0 )
    {
        INFO_STREAM("=====================================");
        INFO_STREAM("Configured robot status object");
    }
}

void StemTrackConfigurer::configureStemTrackController(tue::Configuration& config, StemTrackController& stem_track_controller)
{
    stem_track_controller.setMaxZvelocity( getConfigPar<float>(config, "max_z_dot") );
    stem_track_controller.setUpdateRate( getConfigPar<int>(config, "update_rate") );
    stem_track_controller.setTiltWithStem( getConfigPar<bool>(config, "tilt_with_stem") );
    stem_track_controller.setDebugIKsolver( getConfigPar<bool>(config, "debug_ik_solver") );
    stem_track_controller.setUseInverseVelocitySolverOnly( getConfigPar<bool>(config, "ik_vel_only") );

    if( getLoglevel(config) > 0 )
    {
        INFO_STREAM("=============================================");
        INFO_STREAM("Configured stem track controller object");
    }
}

void StemTrackConfigurer::configureRobotInterface(tue::Configuration& config, RobotInterface& robot_interface)
{
    robot_interface.connectToAmigoArm( getUseLeft(config) );
    robot_interface.connectToAmigoTorso();
    robot_interface.connectToWhiskers();

    if( getLoglevel(config) > 0 )
    {
        INFO_STREAM("=============================================");
        INFO_STREAM("Configured robot interface object");
    }
}

void StemTrackConfigurer::configureStemTrackMonitor(tue::Configuration& config, StemTrackMonitor& stemtrack_monitor)
{
    stemtrack_monitor.setDebugStateParameter( getConfigPar<bool>(config, "debug_state_par") );

    if( getLoglevel(config) > 0 )
    {
        INFO_STREAM("=============================================");
        INFO_STREAM("Configured stemtrack monitor object");
    }
}

void StemTrackConfigurer::configureVisualizationInterface(tue::Configuration& config, VisualizationInterface& visualization_interface)
{
    visualization_interface.connectToRos( getConfigPar<std::string>(config, "topic_name"), getConfigPar<int>(config, "buffer_size") );

    if( getLoglevel(config) > 0 )
    {
        INFO_STREAM("=============================================");
        INFO_STREAM("Configured visualization object");
        INFO_STREAM("=============================================");

    }
}

StemTrackConfigurer::~StemTrackConfigurer()
{
    //destructor
}
