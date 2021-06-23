
#include "robotis_controller/robotis_controller.h"
#include "alice_base_module/base_module.h"
#include "alice_action_module/action_module.h"
#include "alice_tuning_module/tuning_module.h"
#include "alice_online_walking_module/online_walking_module.h"
#include "alice_walking_test_module/walking_test_module.h"
#include "alice_upper_body_module/alice_upper_body_module.h"
#include "alice_torso_module/alice_torso_module.h"
//#include "alice_torso_test_module/alice_torso_test_module.h"

using namespace alice;

#include <iostream>
#include <string>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "alice_Manager");
    ros::NodeHandle nh;


    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

    /* Load ROS Parameter */
    std::string offset_file = nh.param<std::string>("offset_file_path", "");
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");
    std::string init_file   = nh.param<std::string>("init_file_path", "");
    int ip_from_launch  = nh.param<int>("alice_userid",0);
    
    /* gazebo simulation */

    controller->gazebo_mode_ = nh.param<bool>("/gazebo/enable_ros_network", false);
    if(controller->gazebo_mode_ == true)
    {
        ROS_WARN("SET TO GAZEBO MODE!");
        std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
        if(robot_name != "")
            controller->gazebo_robot_name_ = robot_name;
    }


    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
        controller->loadOffset(offset_file);

    sleep(1);

    controller->addMotionModule((robotis_framework::MotionModule*)BaseModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)ActionModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)TuningModule::getInstance());

    //controller->addMotionModule((robotis_framework::MotionModule*)OnlineWalkingModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)WalkingModule::getInstance());

    controller->addMotionModule((robotis_framework::MotionModule*)UpperBodyModule::getInstance());

    controller->addMotionModule((robotis_framework::MotionModule*)TorsoModule::getInstance());
    //controller->addMotionModule((robotis_framework::MotionModule*)TorsoTestModule::getInstance());

    //controller->DEBUG_PRINT = true;
    controller->startTimer();

    while(ros::ok())
    {
      usleep(1000*1000);
    }

    return 0;
}



