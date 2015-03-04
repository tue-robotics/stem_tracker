#include "stemtrackconfigurer.h"

#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "robotinterface.h"
#include "whiskergripperinterpreter.h"
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

const int StemTrackConfigurer::getConfigArrayLength(tue::Configuration& config, const std::string& name)
{
    int n = 0;
    if (config.readArray(name))
    {
        while(config.nextArrayItem())
            ++n;

        config.endArray();
    }
    else
    {
        ERROR_STREAM("Requested array " << name << " not found in config.");
    }

    return n;
}

const std::string StemTrackConfigurer::getBaseFrame(tue::Configuration& config)
{
    return getConfigPar<std::string>(config, "base_frame");
}

const int StemTrackConfigurer::getUpdateRate(tue::Configuration& config)
{
    return getConfigPar<int>(config, "update_rate");
}

const int StemTrackConfigurer::getNumberOfWhiskers(tue::Configuration& config)
{
    std::vector<int> n_whiskers_per_unit;
    float dummy; bool dummy2;
    if (config.readArray("whisker_coverage"))
    {
        while(config.nextArrayItem())
        {
            dummy = getConfigPar<float>(config, "min");
            dummy = getConfigPar<float>(config, "max");
            n_whiskers_per_unit.push_back( getConfigPar<int>(config, "n_whiskers") );
            dummy2 = getConfigPar<bool>(config, "grasp_check");
        }
        config.endArray();
    }

    uint n_whiskers = 0;
    for(uint i = 0; i < n_whiskers_per_unit.size(); ++i)
        n_whiskers += n_whiskers_per_unit[i];

    return n_whiskers;

}

const bool StemTrackConfigurer::getUseLeft(tue::Configuration& config)
{
    return getConfigPar<bool>(config, "use_leftarm");
}

void StemTrackConfigurer::configureStemRepresentation(tue::Configuration& config, StemRepresentation& stem_representation)
{
    stem_representation.setLinTangentDistance( getConfigPar<float>(config, "lin_tan_d") );
    stem_representation.setStemTrackingStartHeight( getConfigPar<float>(config, "start_tracking_at_z") );

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


    INFO_STREAM("===================================================");
    INFO_STREAM("Configured stem representation object for stem " << stem_representation.getStemID() );
    stem_representation.printAll();

}

void StemTrackConfigurer::configureWhiskerGripperInterpreter(tue::Configuration& config, WhiskerGripperInterpreter& whisker_gripper_interpreter)
{
    whisker_gripper_interpreter.setNumberOfWhiskerUnits( getConfigArrayLength(config, "whisker_coverage") );
    whisker_gripper_interpreter.setNumberOfPressureSensors( getConfigArrayLength(config, "pressure_sensor_location" ) );

    whisker_gripper_interpreter.setWhiskerLength( getConfigPar<float>(config, "whisker_length") );
    whisker_gripper_interpreter.setGripperDiameter( getConfigPar<float>(config, "gripper_diameter") );
    whisker_gripper_interpreter.setNumberOfSamplesForInitialization( (int) (getConfigPar<float>(config,"n_seconds_for_initialization") *
                                                                     (float)getConfigPar<int>(config,"update_rate")) );

    std::vector<float> min,max;
    std::vector<bool> grasper;
    std::vector<int> n_whiskers_per_unit;
    if (config.readArray("whisker_coverage"))
    {
        while(config.nextArrayItem())
        {
            min.push_back( getConfigPar<float>(config, "min") );
            max.push_back( getConfigPar<float>(config, "max") );
            n_whiskers_per_unit.push_back( getConfigPar<int>(config, "n_whiskers") );
            grasper.push_back( getConfigPar<bool>(config, "grasp_check") );
        }
        config.endArray();
    }
    whisker_gripper_interpreter.setWhiskerUnitCoversAreaMin(min);
    whisker_gripper_interpreter.setWhiskerUnitCoversAreaMax(max);
    whisker_gripper_interpreter.setWhiskerUnitIsGraspCheck(grasper);
    whisker_gripper_interpreter.setNumberOfWhiskersPerUnit(n_whiskers_per_unit);
    whisker_gripper_interpreter.setNumberOfSamplesForMovingAverage( (int)(getConfigPar<float>(config,"n_sec_for_moving_av")
                                                                 *(float)getConfigPar<int>(config,"update_rate")), getNumberOfWhiskers(config) );
    whisker_gripper_interpreter.setNumberOfWhiskers( getNumberOfWhiskers(config) );

    std::vector<float> touched_max;
    if (config.readArray("whisker_touched_max"))
    {
        while(config.nextArrayItem())
            touched_max.push_back( getConfigPar<float>(config,"max") );
        config.endArray();
    }
    whisker_gripper_interpreter.setWhiskerTouchedMax( touched_max );
    whisker_gripper_interpreter.setWhiskerTouchedNormalizedThreshold( getConfigPar<float>(config,"whisker_touched_normalized_threshold") );
    whisker_gripper_interpreter.setPressureSensorTouchedNormalizedThreshold( getConfigPar<float>(config,"pressure_sensor_touched_normalized_threshold") );

    min.clear(); max.clear();
    if (config.readArray("pressure_sensor_strip_coverage"))
    {
        while(config.nextArrayItem())
        {
            min.push_back( getConfigPar<float>(config, "min") );
            max.push_back( getConfigPar<float>(config, "max") );
        }
        config.endArray();
    }
    whisker_gripper_interpreter.setPressureSensorCoversMax(max);
    whisker_gripper_interpreter.setPressureSensorCoversMin(min);

    std::vector<float> ang;
    if (config.readArray("pressure_sensor_location"))
    {
        while(config.nextArrayItem())
            ang.push_back( getConfigPar<float>(config, "ang") );

        config.endArray();
    }
    whisker_gripper_interpreter.setPressureSensorsAt(ang);

    touched_max.clear();
    if( config.readArray("pressure_sensor_touched_max"))
    {
        while(config.nextArrayItem())
            touched_max.push_back( getConfigPar<float>(config, "max") );
        config.endArray();
    }
    whisker_gripper_interpreter.setPressureSensorTouchedMax(touched_max);

    INFO_STREAM("=====================================================");
    INFO_STREAM("Configured whiskergripper interpreter object" );

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


    INFO_STREAM("====================================================");
    INFO_STREAM("Configured robot representation object of robot " << robot_representation.getName().c_str() );
    robot_representation.printAll();

}

void StemTrackConfigurer::configureRobotStatus(tue::Configuration& config, RobotStatus& robot_status)
{
    robot_status.setXYZreachedThreshold( getConfigPar<double>(config, "xyz_reached_threshold") );
    robot_status.setJointsUpToDateThreshold( getConfigPar<double>(config, "joints_up_to_date_threshold") );
    robot_status.setWhiskersUpToDateThreshold( getConfigPar<double>(config, "whiskers_up_to_date_threshold") );
    robot_status.setPressureSensorsUpToDateThreshold( getConfigPar<double>(config, "pressure_sensors_up_to_date_threshold") );
    robot_status.setPosReachedThreshold( getConfigPar<double>(config, "pos_reached_threshold") );
    robot_status.setNumberOfPressureSensors( getConfigArrayLength(config, "pressure_sensor_location") );
    robot_status.setNumberOfWhiskers( getNumberOfWhiskers( config ) );


    INFO_STREAM("=====================================");
    INFO_STREAM("Configured robot status object");

}

void StemTrackConfigurer::configureStemTrackController(tue::Configuration& config, StemTrackController& stem_track_controller)
{
    stem_track_controller.setMoveUpRef( getConfigPar<float>(config, "move_up_ref") );
    stem_track_controller.setUpdateRate( getConfigPar<int>(config, "update_rate") );
    stem_track_controller.setTiltWithStem( getConfigPar<bool>(config, "tilt_with_stem") );
    stem_track_controller.setDebugIKsolver( getConfigPar<bool>(config, "debug_ik_solver") );
    stem_track_controller.setUseInverseVelocitySolverOnly( getConfigPar<bool>(config, "ik_vel_only") );
    stem_track_controller.setStraightForwardRef( getConfigPar<float>(config, "straight_forward_ref") );

    INFO_STREAM("=============================================");
    INFO_STREAM("Configured stem track controller object");

}

void StemTrackConfigurer::configureRobotInterface(tue::Configuration& config, RobotInterface& robot_interface)
{
    robot_interface.connectToAmigoArm( getUseLeft(config) );
    robot_interface.connectToAmigoTorso();
    robot_interface.connectToWhiskers();
    robot_interface.connectToPressureSensors();


    INFO_STREAM("=============================================");
    INFO_STREAM("Configured robot interface object");

}

void StemTrackConfigurer::configureStemTrackMonitor(tue::Configuration& config, StemTrackMonitor& stemtrack_monitor)
{
    stemtrack_monitor.setDebugStateParameter( getConfigPar<bool>(config, "debug_state_par") );
    stemtrack_monitor.setFindMaxTouchedValues( getConfigPar<bool>(config, "find_max_touched_values") );

    INFO_STREAM("=============================================");
    INFO_STREAM("Configured stemtrack monitor object");

}

void StemTrackConfigurer::configureVisualizationInterface(tue::Configuration& config, VisualizationInterface& visualization_interface)
{
    visualization_interface.connectToRos( getConfigPar<int>(config, "buffer_size") );
    visualization_interface.setShowWhiskerArrowLifetime( getConfigPar<float>(config, "touched_arrow_liftime") );
    visualization_interface.setShowSetPointLifetime( getConfigPar<float>(config, "show_setpoint_lifetime") );
    visualization_interface.setShowStemTangentLifetime( getConfigPar<float>(config, "show_stem_tangent_lifetime") );

    INFO_STREAM("=============================================");
    INFO_STREAM("Configured visualization object");
    INFO_STREAM("=============================================");

}

StemTrackConfigurer::~StemTrackConfigurer()
{
    //destructor
}
