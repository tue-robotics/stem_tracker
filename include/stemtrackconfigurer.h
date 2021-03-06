#ifndef CONFIGURER_H
#define CONFIGURER_H

#include <string>
#include <vector>

#include <tue/config/configuration.h>

/* forward declaration */
class StemRepresentation;
class RobotRepresentation;
class RobotStatus;
class RobotInterface;
class WhiskerGripperInterpreter;
class StemTrackController;
class StemTrackMonitor;
class VisualizationInterface;

class StemTrackConfigurer
{

private:
    tue::Configuration m_general_config;
    template <class T>
    const T getConfigPar(tue::Configuration& config, const std::string& name);
    const int getConfigArrayLength(tue::Configuration& config, const std::string& name);
    std::string m_default_config_path;
    void storeVectorInYmlFile(const std::string& file_name, const std::string& vector_name,
                                                   const std::string& item_name, const std::vector<float>& vector);

public:

    void loadConfig(const int argc, char** argv, const std::string& default_config_path, const std::string& default_config_file);
    void configureStemTrackController(StemTrackController& stem_track_controller);
    void configureStemRepresentation(StemRepresentation& stem);
    void configureRobotRepresentation(RobotRepresentation& robot_representation);
    void configureRobotStatus(RobotStatus& robot_status);
    void configureWhiskerGripperInterpreter(WhiskerGripperInterpreter& whisker_gripper_interpreter);
    void configureRobotInterface(RobotInterface& robot_interface);
    void configureStemTrackMonitor(StemTrackMonitor& stemtrack_monitor);
    void configureVisualizationInterface(VisualizationInterface& visualization_interface);

    const bool configChanged() { return m_general_config.sync(); }
    const bool configIsOk() const;
    const int getUpdateRate();
    const std::string getBaseFrame();
    const bool useLeftArm();
    const int getNumberOfWhiskers();
    const bool debugDesiredGripperPose();
    const float getAdditionalStemHeight() { return getConfigPar<float>(m_general_config, "additional_stem_height_when_done"); }

    void storePressureSensorTouchedMaxValues(const std::vector<float>& pressure_sensor_touched_max);
    void storeWhiskerTouchedMaxValues(const std::vector<float>& whiskers_touched_max);
    void loadPressureSensorTouchedMaxValues(WhiskerGripperInterpreter& whisker_gripper_interpreter);
    void loadWhiskerTouchedMaxValues(WhiskerGripperInterpreter& whisker_gripper_interpreter);

    ~StemTrackConfigurer();

};

#endif // CONFIGURER_H
