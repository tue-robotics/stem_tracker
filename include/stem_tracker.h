#ifndef STEM_TRACKER_H
#define STEM_TRACKER_H

/* c++ includes */
#include <vector>
#include <iostream>

/* ros includes */
#include <ros/ros.h>
#include <ros/package.h>

/* amigo includes */
#include <profiling/StatsPublisher.h>
#include <tue/config/configuration.h>

/* stem tracking includes */
#include "debugfunctions.h"
#include "whiskerinterpreter.h"
#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "visualizationinterface.h"
#include "robotinterface.h"
#include "stemtrackcontroller.h"
#include "stemtrackmonitor.h"
#include "stemtrackconfigurer.h"
#include "loggingmacros.h"

#define     THIS_PACKAGE    "stem_tracker"

#endif // STEM_TRACKER_H
