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

class Configurer
{

private:

    const double extractDouble(tue::Configuration& config, const std::string& name);
    const float extractFloat(tue::Configuration& config, const std::string& name);
    const int extractInt(tue::Configuration& config, const std::string& name);
    const bool extractBool(tue::Configuration& config, const std::string& name);
    const std::string extractString(tue::Configuration& config, const std::string& name);

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
    const int getLoglevel(tue::Configuration& config);
    const bool getUseLeft(tue::Configuration& config);

    ~Configurer();

};

#endif // CONFIGURER_H
