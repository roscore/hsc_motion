#ifndef ALICE_WALKING_MODULE_WALKING_MODULE_H_
#define ALICE_WALKING_MODULE_WALKING_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <boost/thread.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

#include "alice_walking_test_module/alice_walking_test.h"
#include "robotis_framework_common/motion_module.h"

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "alice_walking_module_msgs/GetReferenceStepData.h"
#include "alice_walking_module_msgs/AddStepDataArray.h"
#include "alice_walking_module_msgs/StartWalking.h"
#include "alice_walking_module_msgs/IsRunning.h"
#include "alice_walking_module_msgs/RemoveExistingStepData.h"
#include "alice_walking_module_msgs/SetBalanceParam.h"
#include "alice_walking_module_msgs/SetJointFeedBackGain.h"
#include "alice_ft_sensor_msgs/ForceTorque.h"

namespace alice
{
class WalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<WalkingModule>
{
public:
  WalkingModule();
  virtual ~WalkingModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void onModuleEnable();
  void onModuleDisable();

  void stop();
  bool isRunning();

  void readKinematicsYamlData();
  double leg_to_end_;

private:
  
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);


  /* ROS Topic Callback Functions */
  void imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void ftDataOutputCallback(const alice_ft_sensor_msgs::ForceTorque::ConstPtr &msg);


  void bodysumInitializeCallback(const geometry_msgs::Vector3::ConstPtr &msg);

  /* ROS Service Callback Functions */
  bool setBalanceParamServiceCallback(alice_walking_module_msgs::SetBalanceParam::Request  &req,
      alice_walking_module_msgs::SetBalanceParam::Response &res);

  bool setJointFeedBackGainServiceCallback(alice_walking_module_msgs::SetJointFeedBackGain::Request  &req,
      alice_walking_module_msgs::SetJointFeedBackGain::Response &res);

  bool getReferenceStepDataServiceCallback(alice_walking_module_msgs::GetReferenceStepData::Request  &req,
      alice_walking_module_msgs::GetReferenceStepData::Response &res);
  bool addStepDataServiceCallback(alice_walking_module_msgs::AddStepDataArray::Request  &req,
      alice_walking_module_msgs::AddStepDataArray::Response &res);
  bool startWalkingServiceCallback(alice_walking_module_msgs::StartWalking::Request  &req,
      alice_walking_module_msgs::StartWalking::Response &res);
  bool IsRunningServiceCallback(alice_walking_module_msgs::IsRunning::Request  &req,
      alice_walking_module_msgs::IsRunning::Response &res);
  bool removeExistingStepDataServiceCallback(alice_walking_module_msgs::RemoveExistingStepData::Request  &req,
      alice_walking_module_msgs::RemoveExistingStepData::Response &res);

  int convertStepDataMsgToStepData(alice_walking_module_msgs::StepData& src, robotis_framework::StepData& des);
  int convertStepDataToStepDataMsg(robotis_framework::StepData& src, alice_walking_module_msgs::StepData& des);

  void setBalanceParam(alice_walking_module_msgs::BalanceParam& balance_param_msg);

  void updateBalanceParam();

  bool checkBalanceOnOff();

  void queueThread();

  void setJointFeedBackGain(alice_walking_module_msgs::JointFeedBackGain& msg);
  void updateJointFeedBackGain();

 
  std::map<std::string, int> joint_name_to_index_;

  bool            gazebo_;
  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::mutex    process_mutex_;

  bool previous_running_, present_running;

  ros::Publisher status_msg_pub_;
  ros::Publisher done_msg_pub_;

  // Balance
  bool balance_update_with_loop_;
  double balance_update_duration_;
  double balance_update_sys_time_;
  alice_walking_module_msgs::BalanceParam previous_balance_param_;
  alice_walking_module_msgs::BalanceParam current_balance_param_;
  alice_walking_module_msgs::BalanceParam desired_balance_param_;
  robotis_framework::FifthOrderPolynomialTrajectory balance_update_tra_;

  // Joint
  bool joint_feedback_update_with_loop_;
  double joint_feedback_update_duration_;
  double joint_feedback_update_sys_time_;
  alice_walking_module_msgs::JointFeedBackGain previous_joint_feedback_gain_;
  alice_walking_module_msgs::JointFeedBackGain current_joint_feedback_gain_;
  alice_walking_module_msgs::JointFeedBackGain desired_joint_feedback_gain_;
  robotis_framework::FifthOrderPolynomialTrajectory joint_feedback_update_tra_;

  //Publisher for GUI  

  // refer body, zmp
  ros::Publisher reference_zmp_pub_;
  ros::Publisher reference_body_pub_;
  ros::Publisher reference_body_delta_pub_;
  geometry_msgs::Point reference_zmp_msg_;
  geometry_msgs::Twist reference_body_msg_;
  geometry_msgs::Twist reference_body_delta_msg_;
  bool publish_flag_reference_body_localization;


  // ft calc zmp 
  void FTBasedZmpCalculate(Eigen::Matrix4d g_right_foot, Eigen::Matrix4d g_left_foot, Eigen::MatrixXd g_right_force, Eigen::MatrixXd g_left_force , Eigen::MatrixXd g_right_torque, Eigen::MatrixXd g_left_torque);
  ros::Publisher ft_calc_zmp_pub_;
  geometry_msgs::Point ft_calc_zmp_msg;

  // foot position
  ros::Publisher foot_right_pub_;
  ros::Publisher foot_left_pub_;
  geometry_msgs::Point foot_right_msg_;
  geometry_msgs::Point foot_left_msg_;

  // IMU sensor position
  ros::Publisher angle_sensor_pub_;
  ros::Publisher angle_acc_sensor_pub_;
  geometry_msgs::Vector3 angle_sensor_msg_;
  geometry_msgs::Vector3 angle_acc_sensor_msg_;

  void readIDData();
  std::string alice_id_;

};

}

#endif /* ALICE_WALKING_MODULE_WALKING_MODULE_H_ */
