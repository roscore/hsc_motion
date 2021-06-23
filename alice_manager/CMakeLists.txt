################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(alice_manager)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math
  cmake_modules
  heroehs_math
  
  alice_base_module
  alice_action_module
  alice_tuning_module
  alice_online_walking_module
  alice_walking_test_module
  alice_upper_body_module
  alice_torso_module
  alice_torso_test_module
  #alice_balance_control
)

find_package(Eigen3 REQUIRED)

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
  LIBRARIES alice_manager
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
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


add_executable(alice_manager_node src/alice_manager_node.cpp)

add_dependencies(alice_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(alice_manager_node
  ${catkin_LIBRARIES}
)
