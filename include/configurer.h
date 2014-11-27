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

class Configurer
{

private:

    const double extractDouble(const tue::Configuration& config, const std::string& name);
    const float extractFloat(const tue::Configuration& config, const std::string& name);
    const int extractInt(const tue::Configuration& config, const std::string& name);
    const bool extractBool(const tue::Configuration& config, const std::string& name);
    const std::string extractString(const tue::Configuration& config, const std::string& name);

public:

    void configureStemTrackController(const tue::Configuration& config, StemTrackController& stem_track_controller);
    void configureStemRepresentation(tue::Configuration& config, StemRepresentation& stem);
    void configureRobotRepresentation(tue::Configuration& config, RobotRepresentation& robot_representation);
    void configureRobotStatus(const tue::Configuration& config, RobotStatus& robot_status);
    void configureWhiskerInterpreter(const tue::Configuration& config, WhiskerInterpreter& whisker_interpreter);
    void configureRobotInterface(const tue::Configuration& config, RobotInterface& robot_interface);
    void configureStemTrackMonitor(const tue::Configuration& config, StemTrackMonitor& stemtrack_monitor);

    const int getUpdateRate(const tue::Configuration& config);
    const std::string getBaseFrame(const tue::Configuration& config);
    const int getLoglevel(const tue::Configuration& config);
    const bool getUseLeft(const tue::Configuration& config);

    ~Configurer();

};

#endif // CONFIGURER_H
