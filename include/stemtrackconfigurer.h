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
class WhiskerInterpreter;
class StemTrackController;
class StemTrackMonitor;
class VisualizationInterface;

class StemTrackConfigurer
{

private:
    template <class T>
    const T getConfigPar(tue::Configuration& config, const std::string& name);
    const int getConfigArrayLength(tue::Configuration& config, const std::string& name);

public:

    void configureStemTrackController(tue::Configuration& config, StemTrackController& stem_track_controller);
    void configureStemRepresentation(tue::Configuration& config, StemRepresentation& stem);
    void configureRobotRepresentation(tue::Configuration& config, RobotRepresentation& robot_representation);
    void configureRobotStatus(tue::Configuration& config, RobotStatus& robot_status);
    void configureWhiskerInterpreter(tue::Configuration& config, WhiskerInterpreter& whisker_interpreter);
    void configureRobotInterface(tue::Configuration& config, RobotInterface& robot_interface);
    void configureStemTrackMonitor(tue::Configuration& config, StemTrackMonitor& stemtrack_monitor);
    void configureVisualizationInterface(tue::Configuration& config, VisualizationInterface& visualization_interface);

    const int getUpdateRate(tue::Configuration& config);
    const std::string getBaseFrame(tue::Configuration& config);
    const bool getUseLeft(tue::Configuration& config);

    ~StemTrackConfigurer();

};

#endif // CONFIGURER_H
