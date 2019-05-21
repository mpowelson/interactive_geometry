cmake_minimum_required(VERSION 2.8.3)
project(interactive_geometry_examples)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros, libraries, and packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  interactive_geometry_msgs
  roscpp
  roslib
)

###################################
## catkin specific configuration ##
###################################

# The catkin_package macro generates cmake config files for the package
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    actionlib
    interactive_geometry_msgs
    roscpp
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(interactive_ellipsoid_client_node
  src/interactive_ellipsoid_client_node.cpp
)
add_dependencies(interactive_ellipsoid_client_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(interactive_ellipsoid_client_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executable for installation
install(TARGETS
    interactive_ellipsoid_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
