################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(alice_online_walking_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  robotis_framework_common
  robotis_controller_msgs
  heroehs_online_walking_pattern_generator
  heroehs_pd_balance_controller
  alice_kinematics_dynamics
  alice_walking_module_msgs
  heroehs_math
)

find_package(Eigen REQUIRED)


################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES alice_online_walking_module
  CATKIN_DEPENDS roscpp heroehs_online_walking_pattern_generator heroehs_pd_balance_controller alice_kinematics_dynamics
  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

find_program(UNAME_EXEC uname)
execute_process(COMMAND ${UNAME_EXEC} -m 
    OUTPUT_VARIABLE UNAME_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -rs
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message("SYSTEM INFO ::  ${LSB_RELEASE_ID_SHORT} || ${UNAME_ID_SHORT}")

if (${LSB_RELEASE_ID_SHORT} MATCHES "18.04")
  if ( NOT (${UNAME_ID_SHORT} MATCHES "aarch64")) 
  message( ">>> Linux 18.04 && Not aarch")

  LINK_DIRECTORIES(
    /home/$ENV{USER}/scilab-5.5.2/lib/scilab
    /home/$ENV{USER}/scilab-5.5.2/lib/thirdparty
    /home/$ENV{USER}/scilab-5.5.2/lib/thirdparty/redist
  )
  endif()
endif()

add_library(alice_online_walking_module
  src/alice_online_walking.cpp
  src/online_walking_module.cpp
)

add_dependencies(alice_online_walking_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(alice_online_walking_module
  ${catkin_LIBRARIES}
)
