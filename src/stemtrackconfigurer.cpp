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

#include <tue/config/yaml_emitter.h>
#include <fstream>


template <class T>
const T StemTrackConfigurer::getConfigPar(tue::Configuration& config, const std::string& name)
{
    T tmp;

    if(!config.value(name, tmp))
        ERROR_STREAM("Could not load config value: " << name);

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

const std::string StemTrackConfigurer::getBaseFrame()
{
    return getConfigPar<std::string>(m_general_config, "base_frame");
}

const int StemTrackConfigurer::getUpdateRate()
{
    return getConfigPar<int>(m_general_config, "update_rate");
}

const int StemTrackConfigurer::getNumberOfWhiskers()
{
    std::vector<int> n_whiskers_per_unit;
    float dummy; bool dummy2;
    if (m_general_config.readArray("whisker_coverage"))
    {
        while(m_general_config.nextArrayItem())
        {
            dummy = getConfigPar<float>(m_general_config, "min");
            dummy = getConfigPar<float>(m_general_config, "max");
            n_whiskers_per_unit.push_back( getConfigPar<int>(m_general_config, "n_whiskers") );
            dummy2 = getConfigPar<bool>(m_general_config, "grasp_check");
        }
        m_general_config.endArray();
    }

    uint n_whiskers = 0;
    for(uint i = 0; i < n_whiskers_per_unit.size(); ++i)
        n_whiskers += n_whiskers_per_unit[i];

    return n_whiskers;

}

const bool StemTrackConfigurer::useLeftArm()
{
    return getConfigPar<bool>(m_general_config, "use_leftarm");
}

void StemTrackConfigurer::loadConfig(const int argc, char** argv, const std::string& default_config_path, const std::string& default_config_file)
{
    m_default_config_path = default_config_path;
    /* load configuration */
    if (argc >= 2)
        m_general_config.loadFromYAMLFile(argv[1]);
    else
        m_general_config.loadFromYAMLFile(default_config_path + default_config_file);

    return;
}

const bool StemTrackConfigurer::configIsOk() const
{
    if (m_general_config.hasError())
    {
        ERROR_STREAM("Error in m_general_config" << m_general_config.error());
        return false;
    }
    else
    {
        return true;
    }
}

void StemTrackConfigurer::configureStemRepresentation(StemRepresentation& stem_representation)
{
    stem_representation.setLinTangentDistance( getConfigPar<float>(m_general_config, "lin_tan_d") );
    std::vector<float> stem_start_xyz;
    if( m_general_config.readArray("stem_start_xyz"))
    {
        while(m_general_config.nextArrayItem())
        {
            stem_start_xyz.push_back( getConfigPar<float>(m_general_config, "x") );
            stem_start_xyz.push_back( getConfigPar<float>(m_general_config, "y") );
            stem_start_xyz.push_back( getConfigPar<float>(m_general_config, "z") );
        }
        m_general_config.endArray();
    }
    stem_representation.setStemTrackingStartXYZ( stem_start_xyz );

    stem_representation.setAddOrRemoveNodeThreshold( getConfigPar<float>(m_general_config, "add_or_remove_node_euclidian_threshold"));

    std::vector<float> stemNodesX, stemNodesY, stemNodesZ;

    /* store initial stem nodes on stem_start_xyz and directly below */
    stemNodesX.push_back(stem_start_xyz[0]);
    stemNodesX.push_back(stem_start_xyz[0]);
    stemNodesY.push_back(stem_start_xyz[1]);
    stemNodesY.push_back(stem_start_xyz[1]);
    stemNodesZ.push_back(stem_start_xyz[2]-getConfigPar<float>(m_general_config,"length_first_node"));
    stemNodesZ.push_back(stem_start_xyz[2]);

    stem_representation.loadNodesXYZ( stemNodesX, stemNodesY, stemNodesZ);

    if( !useLeftArm() )
        stem_representation.flipNodes();


    INFO_STREAM("===================================================");
    INFO_STREAM("Configured stem representation object for stem " << stem_representation.getStemID() );
    stem_representation.printAll();

}

void StemTrackConfigurer::configureWhiskerGripperInterpreter(WhiskerGripperInterpreter& whisker_gripper_interpreter)
{
    whisker_gripper_interpreter.setNumberOfWhiskerUnits( getConfigArrayLength(m_general_config, "whisker_coverage") );
    whisker_gripper_interpreter.setNumberOfPressureSensors( getConfigArrayLength(m_general_config, "pressure_sensor_location" ) );

    whisker_gripper_interpreter.setWhiskerLength( getConfigPar<float>(m_general_config, "whisker_length") );
    whisker_gripper_interpreter.setGripperDiameter( getConfigPar<float>(m_general_config, "gripper_diameter") );
    whisker_gripper_interpreter.setNumberOfSamplesForInitialization( (int) (getConfigPar<float>(m_general_config,"n_seconds_for_initialization") *
                                                                     (float)getConfigPar<int>(m_general_config,"update_rate")) );

    std::vector<float> min,max;
    std::vector<bool> grasper;
    std::vector<int> n_whiskers_per_unit;
    if (m_general_config.readArray("whisker_coverage"))
    {
        while(m_general_config.nextArrayItem())
        {
            min.push_back( getConfigPar<float>(m_general_config, "min") );
            max.push_back( getConfigPar<float>(m_general_config, "max") );
            n_whiskers_per_unit.push_back( getConfigPar<int>(m_general_config, "n_whiskers") );
            grasper.push_back( getConfigPar<bool>(m_general_config, "grasp_check") );
        }
        m_general_config.endArray();
    }
    whisker_gripper_interpreter.setWhiskerUnitCoversAreaMin(min);
    whisker_gripper_interpreter.setWhiskerUnitCoversAreaMax(max);
    whisker_gripper_interpreter.setWhiskerUnitIsGraspCheck(grasper);
    whisker_gripper_interpreter.setNumberOfWhiskersPerUnit(n_whiskers_per_unit);
    whisker_gripper_interpreter.setNumberOfSamplesForMovingAverage( (int)(getConfigPar<float>(m_general_config,"n_sec_for_moving_av")
                                                                 *(float)getConfigPar<int>(m_general_config,"update_rate")), getNumberOfWhiskers() );
    whisker_gripper_interpreter.setNumberOfWhiskers( getNumberOfWhiskers() );

    whisker_gripper_interpreter.setWhiskerTouchedNormalizedThreshold( getConfigPar<float>(m_general_config,"whisker_touched_normalized_threshold") );
    whisker_gripper_interpreter.setPressureSensorTouchedNormalizedThreshold( getConfigPar<float>(m_general_config,"pressure_sensor_touched_normalized_threshold") );

    min.clear(); max.clear();
    if (m_general_config.readArray("pressure_sensor_strip_coverage"))
    {
        while(m_general_config.nextArrayItem())
        {
            min.push_back( getConfigPar<float>(m_general_config, "min") );
            max.push_back( getConfigPar<float>(m_general_config, "max") );
        }
        m_general_config.endArray();
    }
    whisker_gripper_interpreter.setPressureSensorCoversMax(max);
    whisker_gripper_interpreter.setPressureSensorCoversMin(min);

    std::vector<float> ang;
    if (m_general_config.readArray("pressure_sensor_location"))
    {
        while(m_general_config.nextArrayItem())
            ang.push_back( getConfigPar<float>(m_general_config, "ang") );

        m_general_config.endArray();
    }
    whisker_gripper_interpreter.setPressureSensorsAt(ang);

    if(!getConfigPar<bool>(m_general_config, "find_max_touched_values") )
    {
       loadPressureSensorTouchedMaxValues(whisker_gripper_interpreter);
       loadWhiskerTouchedMaxValues(whisker_gripper_interpreter);
    }

    INFO_STREAM("=====================================================");
    INFO_STREAM("Configured whiskergripper interpreter object" );

}

void StemTrackConfigurer::loadPressureSensorTouchedMaxValues(WhiskerGripperInterpreter& whisker_gripper_interpreter)
{
    std::vector<float> touched_max;
    tue::Configuration config;
    config.loadFromYAMLFile( m_default_config_path + getConfigPar<std::string>(m_general_config, "config_file_pressure_sensor_touched_max") );

    if( config.readArray("pressure_sensor_touched_max"))
    {
        while(config.nextArrayItem())
            touched_max.push_back( getConfigPar<float>(config, "max") );
        config.endArray();
    }
    if(touched_max.size() != getConfigArrayLength(m_general_config, "pressure_sensor_location") )
    {
        ERROR_STREAM("I got " << touched_max.size() << " max pressure sensor values while n_pressure sensors is " <<
                     getConfigArrayLength(m_general_config, "pressure_sensor_location") );
        return;
    }
    whisker_gripper_interpreter.setPressureSensorTouchedMax(touched_max);
}

void StemTrackConfigurer::loadWhiskerTouchedMaxValues(WhiskerGripperInterpreter& whisker_gripper_interpreter)
{
    std::vector<float> touched_max;
    tue::Configuration config;
    config.loadFromYAMLFile( m_default_config_path + getConfigPar<std::string>(m_general_config, "config_file_whiskers_touched_max") );

    if(config.readArray("whiskers_touched_max"))
    {
        while(config.nextArrayItem())
            touched_max.push_back( getConfigPar<float>(config,"max") );
        config.endArray();
    }
    if(touched_max.size() != getNumberOfWhiskers())
    {
        ERROR_STREAM("I got " << touched_max.size() << " max values while n_whiskers is " << getNumberOfWhiskers());
        return;
    }
    whisker_gripper_interpreter.setWhiskerTouchedMax( touched_max );
}

void StemTrackConfigurer::configureRobotRepresentation(RobotRepresentation& robot_representation)
{
    if( useLeftArm() )
        robot_representation.setLeftArmIsPreferred();
    else
        robot_representation.setRightArmIsPreferred();

    robot_representation.loadUrdfFromFile( getConfigPar<std::string>(m_general_config, "robot_urdf_file") );
    robot_representation.loadKinematicTreeFromUrdf();

    if( useLeftArm() )
        robot_representation.loadKinematicChainFromTree( getConfigPar<std::string>(m_general_config, "root_link"), getConfigPar<std::string>(m_general_config, "left_end_link") );
    else
        robot_representation.loadKinematicChainFromTree( getConfigPar<std::string>(m_general_config, "root_link"), getConfigPar<std::string>(m_general_config, "right_end_link") );

    robot_representation.loadJointLimits();

    std::vector<float> tmp;
    if (m_general_config.readArray("initial_pose"))
    {
        while(m_general_config.nextArrayItem())
            tmp.push_back( getConfigPar<float>(m_general_config, "q") );

        m_general_config.endArray();
    }
    robot_representation.setInitialPoseJointRefs( tmp );


    INFO_STREAM("====================================================");
    INFO_STREAM("Configured robot representation object of robot " << robot_representation.getName().c_str() );
    robot_representation.printAll();

}

void StemTrackConfigurer::configureRobotStatus(RobotStatus& robot_status)
{
    robot_status.setXYZreachedThreshold( getConfigPar<double>(m_general_config, "xyz_reached_threshold") );
    robot_status.setJointsUpToDateThreshold( getConfigPar<double>(m_general_config, "joints_up_to_date_threshold") );
    robot_status.setWhiskersUpToDateThreshold( getConfigPar<double>(m_general_config, "whiskers_up_to_date_threshold") );
    robot_status.setPressureSensorsUpToDateThreshold( getConfigPar<double>(m_general_config, "pressure_sensors_up_to_date_threshold") );
    robot_status.setPosReachedThreshold( getConfigPar<double>(m_general_config, "pos_reached_threshold") );
    robot_status.setNumberOfPressureSensors( getConfigArrayLength(m_general_config, "pressure_sensor_location") );
    robot_status.setNumberOfWhiskers( getNumberOfWhiskers() );


    INFO_STREAM("=====================================");
    INFO_STREAM("Configured robot status object");

}

void StemTrackConfigurer::configureStemTrackController(StemTrackController& stem_track_controller)
{
    stem_track_controller.setMoveUpRef( getConfigPar<float>(m_general_config, "move_up_ref") );
    stem_track_controller.setUpdateRate( getConfigPar<int>(m_general_config, "update_rate") );
    stem_track_controller.setTiltWithStem( getConfigPar<bool>(m_general_config, "tilt_with_stem") );
    stem_track_controller.setDebugIKsolver( getConfigPar<bool>(m_general_config, "debug_ik_solver") );
    stem_track_controller.setUseInverseVelocitySolverOnly( getConfigPar<bool>(m_general_config, "ik_vel_only") );
    stem_track_controller.setStraightForwardRef( getConfigPar<float>(m_general_config, "straight_forward_ref") );
    stem_track_controller.setPrintRefVsCurrent( getConfigPar<bool>(m_general_config, "print_joint_ref_vs_current") );
    stem_track_controller.setSetpointMultiplicationAtMaxTorso( getConfigPar<float>(m_general_config, "xyz_setpoint_multiplication_at_max_torso"));

    INFO_STREAM("=============================================");
    INFO_STREAM("Configured stem track controller object");

}

void StemTrackConfigurer::storePressureSensorTouchedMaxValues(const std::vector<float>& pressure_sensor_touched_max)
{
    storeVectorInYmlFile( getConfigPar<std::string>(m_general_config, "config_file_pressure_sensor_touched_max"), "pressure_sensor_touched_max",
                          "max", pressure_sensor_touched_max);
    return;
}

void StemTrackConfigurer::storeWhiskerTouchedMaxValues(const std::vector<float>& whiskers_touched_max)
{
    storeVectorInYmlFile( getConfigPar<std::string>(m_general_config, "config_file_whiskers_touched_max"), "whiskers_touched_max",
                          "max", whiskers_touched_max);
    return;
}

void StemTrackConfigurer::storeVectorInYmlFile(const std::string& file_name, const std::string& vector_name,
                                               const std::string& item_name, const std::vector<float>& vector)
{
    tue::Configuration cfg;
    cfg.writeArray(vector_name);
    for(int i = 0; i < vector.size(); ++i)
    {
        cfg.addArrayItem();
        cfg.setValue(item_name, vector[i]);
        cfg.endArrayItem();
    }
    cfg.endArray();

    std::ofstream f_out;
    std::string tmp = m_default_config_path + file_name;
    f_out.open(tmp.c_str());

    tue::config::YAMLEmitter emitter;
    emitter.emit(cfg.data(), f_out);
    f_out.close();

    return;
}

void StemTrackConfigurer::configureRobotInterface(RobotInterface& robot_interface)
{
    robot_interface.connectToAmigoArm( useLeftArm() );
    robot_interface.connectToAmigoTorso();
    robot_interface.connectToWhiskers();
    robot_interface.connectToPressureSensors();
    robot_interface.connectToAmigoGripper( useLeftArm() );


    INFO_STREAM("=============================================");
    INFO_STREAM("Configured robot interface object");

}

void StemTrackConfigurer::configureStemTrackMonitor(StemTrackMonitor& stemtrack_monitor)
{
    stemtrack_monitor.setDebugStateParameter( getConfigPar<bool>(m_general_config, "debug_state_par") );
    stemtrack_monitor.setFindMaxTouchedValues( getConfigPar<bool>(m_general_config, "find_max_touched_values") );

    INFO_STREAM("=============================================");
    INFO_STREAM("Configured stemtrack monitor object");
    INFO_STREAM("=============================================");
}

void StemTrackConfigurer::configureVisualizationInterface(VisualizationInterface& visualization_interface)
{
    visualization_interface.connectToRos( getConfigPar<int>(m_general_config, "buffer_size") );
    visualization_interface.setShowWhiskerArrowLifetime( getConfigPar<float>(m_general_config, "touched_arrow_liftime") );
    visualization_interface.setShowSetPointLifetime( getConfigPar<float>(m_general_config, "show_setpoint_lifetime") );
    visualization_interface.setShowStemTangentLifetime( getConfigPar<float>(m_general_config, "show_stem_tangent_lifetime") );

    INFO_STREAM("=============================================");
    INFO_STREAM("Configured visualization object");

}

StemTrackConfigurer::~StemTrackConfigurer()
{
    //destructor
}
