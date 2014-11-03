cmake_minimum_required(VERSION 2.8.3)
project(stem_tracker)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    code_profiler
    sensor_msgs
    kdl_parser
    tue_config
    roslib # For using ros::package::getPath
)


# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------


# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

#Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#   std_msgs
#)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES stem_tracker_libs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------


# show the headers in qtcreator
file(GLOB HEADERS
  RELATIVE ${PROJECT_SOURCE_DIR} "include/*.h"
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(stem_tracker_libs
  ${HEADERS}
  src/robotconfig.cpp
  src/robotstatus.cpp
  src/stemrepresentation.cpp
  src/whiskerinterpreter.cpp
  src/visualizationinterface.cpp
  src/debugfunctions.cpp
  src/robotinterface.cpp
  src/stemtrackcontroller.cpp
  src/stemtrackmonitor.cpp
)

target_link_libraries(stem_tracker_libs
     ${catkin_LIBRARIES}
)

add_executable(stem_tracker
    ${HEADERS}
    src/stem_tracker.cpp
)
target_link_libraries(stem_tracker ${catkin_LIBRARIES} stem_tracker_libs)



