cmake_minimum_required(VERSION 2.8.3)
project(stem_tracker)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    code_profiler
    genmsg
    actionlib_msgs
    actionlib
    arm_navigation_msgs
    geometry_msgs
    std_msgs
    tf
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

add_action_files(DIRECTORY ../amigo_whole_body_controller/action FILES ArmTask.action)

add_message_files(DIRECTORY ../amigo_whole_body_controller/msg FILES WholeBodyControllerStatus.msg)

generate_messages(DEPENDENCIES
    actionlib_msgs
    arm_navigation_msgs
    std_msgs
    geometry_msgs
)



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
#    INCLUDE_DIRS include
#  LIBRARIES bla
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

add_executable(stem_tracker
    ${HEADERS}
    src/stem_tracker.cpp
)

target_link_libraries(stem_tracker ${catkin_LIBRARIES} )


