#include "logginginterface.h"

#include <ros/ros.h>

LoggingInterface::LoggingInterface()
{
}

void LoggingInterface::logStream(std::stringstream stream)
{
    ROS_INFO_STREAM(stream);
}
