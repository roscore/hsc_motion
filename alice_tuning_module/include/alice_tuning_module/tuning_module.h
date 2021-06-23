#ifndef ALICE_TUNING_MODULE_TUNING_MODULE_H_
#define ALICE_TUNING_MODULE_TUNING_MODULE_H_

#include <map>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <numeric>
#include <fstream>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/LoadOffset.h"
#include "robotis_math/robotis_math.h"
#include "alice_kinematics_dynamics/kinematics_dynamics.h"

#include "alice_tuning_module_msgs/JointOffsetData.h"
#include "alice_tuning_module_msgs/JointTorqueOnOffArray.h"
#include "alice_tuning_module_msgs/GetPresentJointOffsetData.h"
#include "alice_tuning_module_msgs/JointsOffsetPositionData.h"

#include "tuning_module_state.h"
#include "tuning_data.h"

namespace alice
{

class TuningJointData
{
 public:
  double position_;


};

class TuneJointState
{
 public:
  TuningJointData curr_joint_state_[ MAX_JOINT_ID + 1];
  TuningJointData goal_joint_state_[ MAX_JOINT_ID + 1];
  TuningJointData fake_joint_state_[ MAX_JOINT_ID + 1];

};

class TuningModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<TuningModule>
{
 public:
  TuningModule();
  virtual ~TuningModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

  /* ROS Topic Callback Functions */
  void tunePoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  void commandCallback(const std_msgs::String::ConstPtr& msg);
  void jointOffsetDataCallback(const alice_tuning_module_msgs::JointOffsetData::ConstPtr &msg);
  void jointTorqueOnOffCallback(const alice_tuning_module_msgs::JointTorqueOnOffArray::ConstPtr& msg);
  bool getPresentJointOffsetDataServiceCallback(alice_tuning_module_msgs::GetPresentJointOffsetData::Request &req,
                                                alice_tuning_module_msgs::GetPresentJointOffsetData::Response &res);

  /* ROS Calculation Functions */
  void targetPoseTrajGenerateProc();

  /* Parameter */
  // state for generating trajectory
  TuningModuleState *tuning_module_state_;
  // state of the joints
  TuneJointState *joint_state_;

 private:
  void queueThread();
  void setCtrlModule(std::string module);
  void callServiceSettingModule(const std::string &module_name);


  void moveToTunePose(const std::string &pose_name);
  
  bool parseOffsetData_temp(const std::string &path, const std::string &pose);
  bool parseOffsetData(const std::string &path, const std::string &pose, int dir);
  bool parseTunePoseData(const std::string &path, const std::string &pose, int dir);

  void publishStatusMsg(unsigned int type, std::string msg);
  bool turnOnOffOffset(bool turn_on);
  bool loadOffsetToController(const std::string &path);
  void saveOffsetToYaml(const std::string &path);

  void sendPresentJointOffsetData();

  int control_cycle_msec_;
  boost::thread queue_thread_;
  boost::thread tra_gene_tread_;
  boost::mutex data_mutex_;

  // 
  ros::Publisher status_msg_pub_;
  ros::Publisher set_ctrl_module_pub_;
  ros::ServiceClient set_module_client_;

  // offset tuner
  ros::Publisher sync_write_pub_;
  ros::Publisher enable_offset_pub_;
  ros::Publisher present_joint_pub_;
  ros::Subscriber send_tra_sub_;
  ros::Subscriber joint_offset_data_sub_;
  ros::Subscriber joint_torque_enable_sub_;
  ros::Subscriber command_sub_;
  ros::ServiceServer offset_data_server_;
  ros::ServiceClient load_offset_client_;

  std::map<std::string, int> joint_name_to_id_;
  // data set for tuner client
  std::map<std::string, double> temp_offset_data_;
  std::map<std::string, JointOffsetData*> robot_tuning_data_;
  std::map<std::string, bool> robot_torque_enable_data_;

  std::string offset_path_;
  std::string base_path_;
  // data set for write the dxls
  TuningData tuning_data_;

  bool has_goal_joints_;
  bool ini_pose_only_;
  bool get_tuning_data_;

	std::string alice_id_;

  void readIDData();
  bool previous_offset_mode_;
  bool previous_offset_pose_;
};

}

#endif /* ALICE_TUNING_MODULE_TUNING_MODULE_H_ */
