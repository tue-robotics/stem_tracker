#ifndef LOGGINGINTERFACE_H
#define LOGGINGINTERFACE_H

#include <sstream>

class LoggingInterface
{
public:
    LoggingInterface();

    void logStream(std::stringstream stream);

    ~LoggingInterface();
};

#endif // LOGGINGINTERFACE_H
