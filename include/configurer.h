#ifndef CONFIGURER_H
#define CONFIGURER_H

#define     INFO_STREAM     ROS_INFO_STREAM

#include <string>
#include <vector>

#include <tue/config/configuration.h>

#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "robotinterface.h"
#include "whiskerinterpreter.h"
#include "stemtrackcontroller.h"

class Configurer
{
private:

    /* global config parameters */
    bool g_DEBUG;
    int g_UPDATE_RATE;
    std::string g_BASE_FRAME;

    /* robot representation config parameters */
    std::string rr_ROOT_LINK;
    bool rr_USE_LEFTARM;
    std::string rr_ROBOT_DESCRIPTION_ROSPARAM;
    std::string rr_LEFT_END_LINK;
    std::string rr_RIGHT_END_LINK;

    /* robot status config parameters */
    double rs_UP_TO_DATE_THRESHOLD;

    /* stem representation config parameters */
    float sr_STEM_THICKNESS;
    float sr_STEM_RGB[3];
    std::vector<float> sr_stemNodesX;
    std::vector<float> sr_stemNodesY;
    std::vector<float> sr_stemNodesZ;

    /* whisker interpreter config parameters */
    int wi_N_WHISKERS;
    float wi_GRIPPER_DIAMETER;
    float wi_WHISKER_LENGTH;
    float wi_MAX_WHISKER_FORCE;

    /* stem track controller config parameters */
    float stc_MAX_Z_DOT;

public:

    void loadGlobalConfig(tue::Configuration config);
    void configureStemTrackController(tue::Configuration config, StemTrackController* p_stem_track_controller);
    void configureStemRepresentation(tue::Configuration config, StemRepresentation* p_stem);
    void configureRobotRepresentation(tue::Configuration config, RobotRepresentation* p_robot_representation, ros::NodeHandle n);
    void configureRobotStatus(tue::Configuration config, RobotStatus* p_robot_status);
    void configureWhiskerInterpreter(tue::Configuration config, WhiskerInterpreter* p_whisker_interpreter);
    void configureRobotInterface(tue::Configuration config, RobotInterface* p_robot_interface);
    int getUpdateRate(tue::Configuration config);
    std::string getBaseFrame(tue::Configuration config);

    ~Configurer();
};

#endif // CONFIGURER_H
