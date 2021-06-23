/*
 * alice_torso_module.h
 *
 *  Created on: Aug 10, 2020
 *      Author: heroehs
 */

#ifndef ALICE_TORSO_MODULE_TORSO_MODULE_H_
#define ALICE_TORSO_MODULE_TORSO_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8.h>
#include <alice_msgs/FoundObject.h>
#include <geometry_msgs/Pose.h>
#include <diagnostic_msgs/KeyValue.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <fstream>
#include <stdio.h>
#include <math.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"

#include "alice_msgs/FoundObjectArray.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "pid_controller.h"
#include "fifth_order_trajectory_generator.h"

#define PI 3.141592
#define DEG2RAD_(x) (x * 0.01745329252)  // *PI/180
#define RAD2DEG_(x) (x * 57.2957795131) // *180/PI

namespace alice
{
class TorsoModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<TorsoModule>
{
public:
  TorsoModule();
  virtual ~TorsoModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

// publisher
	ros::Publisher  scan_done_pub;
	ros::Publisher  robot_state_pub;

	// Subscriber
  ros::Subscriber scan_cmd_sub;

	ros::Subscriber head_test;
	ros::Subscriber waist_test;

	ros::Subscriber environment_detector_sub;
	ros::Subscriber detected_objects_sub;
	ros::Subscriber head_moving_sub;
	ros::Subscriber arm_moving_sub;

	ros::Subscriber ball_test_sub;
	ros::Subscriber ball_param_sub;

	ros::Subscriber walking_module_status_sub;

	//msg
	std_msgs::Bool scan_done_msg;

	void desiredPoseWaistMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredPoseHeadMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void environmentDetectorMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg);
	void detectedObjectsMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg);

	void headMovingMsgCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg);
	
	void ballTestMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void ballTestParamMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

	void walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);
	void readIDData();

  void torsomodeCallback(const std_msgs::Int32::ConstPtr& msg);
  void scanCallback(const std_msgs::Bool::ConstPtr& msg);

  double f_cut, w_cut, tau;
  double y, pre_y, pre_x;
  double ts;

  double LowpassFilter(double x);
  bool gazebo_check;
  bool is_moving_state;
  bool update_state;
  
  std::map<std::string, PIDController *> dxl_pidcontroller;
  /* ROS Topic Callback Functions */

  void manualMovingCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void headmanualCallback(const std_msgs::Bool::ConstPtr& msg);
  void headtrackingCallback(const std_msgs::Bool::ConstPtr& msg);
  void headtrackingctrlCallback(const alice_msgs::FoundObject::ConstPtr& msg);
  int error_calculate(int origin_point, int object_point);

  std::map<std::string, alice::FifthOrderTrajectory *> motion_trajectory;
  bool motion_phase_init;
  
private:
  void queueThread();
  bool running_;

  int new_count_;
	bool is_moving_head_;
  int control_cycle_msec_;

  std::string alice_id_;

  uint8_t command;

  double x_p_gain, x_d_gain, y_p_gain, y_d_gain;

	double current_x,current_y;
	double pre_current_x,pre_current_y;
	int frame_x, frame_y;
	double desired_x, desired_y;

	double control_angle_yaw, control_angle_pitch;
	double control_angle_yaw_temp, control_angle_pitch_temp;

	//ball detecting
	bool ball_detected;

	//motion
	double current_time_scanning;
	int motion_num_scanning;

	double current_time_finding;
	int motion_num_finding;

  double current_time_checking;
  int motion_num_checking;

	double current_time_arm_motion;
	int motion_num_arm;

  double head_yaw_goal;
  double head_pitch_goal;

	// detected objects and absolute
	double current_goal_x,current_goal_y;
	double current_center_x,current_center_y;
	double current_robot_x, current_robot_y;
	double current_robot_theta;
	geometry_msgs::Vector3 robot_state_msg;

  int max_limit_[3];

  int mode_;
  int pre_mode_;
  int search_phase;

  float search_motion[6][2];
  float motion_bias;

  int max_boxsize_;
  int min_boxsize_;

  int x_resolution, y_resolution, x_origin, y_origin;
  int roi_x_offset, roi_y_offset, roi_height, roi_width, id;
  double distance;

  double error_[3];
  double max_error_[3];
  double min_error_[3];
  double mapped_error_[3];

  double min_p_gain[3];
  double max_p_gain[3];

  boost::thread queue_thread_;

  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;

  std::map<int, double> joint_id_to_rad_;

  std::map<std::string,double> joint_name_to_curr_pose_;
  std::map<std::string,double> joint_name_to_goal_pose_;

  double mov_time_state;
  std::map<std::string, double> joint_name_to_ini_pose_state_;
  std::map<std::string, double> joint_name_to_ini_pose_goal_;

};
}




#endif /* ALICE_HEROEHS_ALICE_MOTION_ALICE_TORSO_MODULE_INCLUDE_ALICE_TORSO_MODULE_ALICE_TORSO_MODULE_H_ */
