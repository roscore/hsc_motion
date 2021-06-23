#ifndef ALICE_BASE_MODULE_BASE_MODULE_H_
#define ALICE_BASE_MODULE_BASE_MODULE_H_

#include <map>
#include <fstream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/StatusMsg.h"

//
#include "robotis_controller_msgs/LoadOffset.h"
#include <std_msgs/Bool.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "alice_kinematics_dynamics/kinematics_dynamics.h"

#include "base_module_state.h"

namespace alice
{

class BaseJointData
{
public:
  double position_;

};

class BaseJointState
{
public:
  BaseJointData curr_joint_state_[MAX_JOINT_ID + 1];
  BaseJointData goal_joint_state_[MAX_JOINT_ID + 1];
  BaseJointData fake_joint_state_[MAX_JOINT_ID + 1];
};

class BaseModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<BaseModule>
{
public:
  BaseModule();
  virtual ~BaseModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  /* ROS Topic Callback Functions */
  void BasePoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  
  /* ROS Calculation Functions */
  void initPoseTrajGenerateProc();

  //void poseGenerateProc(Eigen::MatrixXd joint_angle_pose);
  //void poseGenerateProc(std::map<std::string, double>& joint_angle_pose);

  /* Parameter */
  BaseModuleState  *base_module_state_;
  BaseJointState   *joint_state_;

private:
  void queueThread();
  
  void setCtrlModule(std::string module);
  void parsePoseData(const std::string &path, const std::string &pose);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(const std::string done_msg);

  int           control_cycle_msec_;

  boost::thread queue_thread_;
  boost::thread tra_gene_tread_;

  // ros::Publisher      send_tra_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher set_ctrl_module_pub_;
  ros::Publisher movement_done_pub_;



  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;

  bool has_goal_joints_;
  bool ini_pose_only_;


	void readIDData();
	std::string alice_id_;

  void ResetBaseModule();
  


};

}

#endif /* ALICE_BASE_MODULE_BASE_MODULE_H_ */
