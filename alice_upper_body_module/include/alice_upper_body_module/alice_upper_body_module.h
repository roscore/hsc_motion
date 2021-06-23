/*
 * alice_upper_body_module.h
 *
 *  Created on: May 30, 2018
 *      Author: robotemperor
 */

#ifndef ALICE_HEROEHS_ALICE_MOTION_ALICE_UPPER_BODY_MODULE_INCLUDE_ALICE_UPPER_BODY_MODULE_H_
#define ALICE_HEROEHS_ALICE_MOTION_ALICE_UPPER_BODY_MODULE_INCLUDE_ALICE_UPPER_BODY_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <float.h> // isnan()

#include "robotis_framework_common/motion_module.h"
//library
#include "robotis_math/robotis_math.h"
#include "heroehs_math/fifth_order_trajectory_generate.h"
#include "heroehs_math/end_point_to_rad_cal.h"

//message
//m - standard
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/KeyValue.h>
//m - personal
#include "alice_msgs/FoundObjectArray.h"
#include "robotis_controller_msgs/StatusMsg.h"


namespace alice
{

class UpperBodyModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<UpperBodyModule>
{
public:
	UpperBodyModule();
	virtual ~UpperBodyModule();

	/* ROS Framework Functions */
	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

	void stop();
	bool isRunning();

	bool gazebo_check;
	bool is_moving_state;

	// Subscriber
	ros::Subscriber arm_moving_sub;

	void armMovingMsgCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg);
	void readIDData();

private:
	void queueThread();
	bool running_;
	int control_cycle_msec_;
  	int flag_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

	//algorithm
	bool arm_motion_run;

	//motion
	double current_time_arm_motion;
	int motion_num_arm;

	// trajectory generation for each dxl
	std::map<std::string, heroehs_math::FifthOrderTrajectory *> motion_trajectory;

	// detected objects and absolute
	std::string alice_id_;

	void parse_motion_data(const std::string &path);
	std::vector<std::string> motion_joint_data_;
	std::map<int, std::vector<double> > motion_numb_to_joint_pose_data_;
	std::vector<double> motion_time_data_;

	double mov_time_state;
	std::map<std::string, double> joint_name_to_ini_pose_state_;
	std::map<std::string, double> joint_name_to_ini_pose_goal_;
};

}

#endif /* ALICE_HEROEHS_ALICE_MOTION_ALICE_UPPER_BODY_MODULE_INCLUDE_ALICE_UPPER_BODY_MODULE_H_ */