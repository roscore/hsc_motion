

#include <stdio.h>
#include "alice_base_module/base_module.h"


using namespace alice;

BaseModule::BaseModule()
  : control_cycle_msec_(0),
    has_goal_joints_(false),
    ini_pose_only_(false)
{
  enable_       = false;
  module_name_  = "base_module";
  control_mode_ = robotis_framework::PositionControl;

  readIDData();

	result_["l_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 11
	result_["l_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 13
	result_["l_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 15
	result_["l_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 17
	result_["l_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 19
	result_["l_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 21

  result_["r_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 12
	result_["r_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 14
	result_["r_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 16
	result_["r_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 18
	result_["r_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 20
	result_["r_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 22

  if(alice_id_ == "1")
  {
    result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
    result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
    result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
    result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
    result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
    result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
    result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6

  }
  else if(alice_id_ == "3")
  {
    result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
    result_["waist_pitch"]      = new robotis_framework::DynamixelState();  // joint 10
    result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
  	result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
    result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
    result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
    result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
    result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6

  }
  else if(alice_id_ == "2")
  {
    result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
    result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
    result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
    result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
    result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
    result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6
  }

	result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 7
	result_["head_pitch"]       = new robotis_framework::DynamixelState();  // joint 8

  

  base_module_state_  = new BaseModuleState();
  joint_state_    = new BaseJointState();

  joint_name_to_id_.clear();
	joint_id_to_name_.clear();
}

BaseModule::~BaseModule()
{
  queue_thread_.join();
}
void BaseModule::readIDData()
{
  ros::NodeHandle nh;
  int alice_id_int  = nh.param<int>("alice_userid",0);

  ROS_INFO("Base id: %d",alice_id_int);
  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  alice_id_ = alice_id_stream.str();
}
void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_       = boost::thread(boost::bind(&BaseModule::queueThread, this));

  ros::NodeHandle ros_node;

  /* publish topics */
  status_msg_pub_       = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_  = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  movement_done_pub_ = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
	}
	ROS_INFO("< -------  Initialize Module : Base Module !!  ------->");


}

void BaseModule::parsePoseData(const std::string &path, const std::string &pose)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  YAML::Node base_pose_node, tar_pose_node ;

  base_pose_node = doc[pose];

  if(base_pose_node == NULL)
  {
    ROS_ERROR("Fail to parse yaml");
    return;
  }

  double mov_time;
  mov_time = base_pose_node["move_time"].as<double>();
  base_module_state_->mov_time_ = mov_time;

  tar_pose_node = base_pose_node["target_pose"];

  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    std::string joint_name;
    double value;

    joint_name = it->first.as<std::string>();

    value = it->second.as<double>();

    base_module_state_->joint_ini_pose_.coeffRef(joint_name_to_id_[joint_name], 0) = value * DEGREE2RADIAN;
  }

  base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;
  base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
}




void BaseModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */

  // for gui
  ros::Subscriber base_pose_msg_sub = ros_node.subscribe("/base_module/pose", 5, &BaseModule::BasePoseMsgCallback, this);
 
  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void BaseModule::BasePoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (base_module_state_->is_moving_ == false)
  {
    if (msg->data == "base_pose" )
    {

      setCtrlModule(module_name_);

      while (enable_ == false || has_goal_joints_ == false)
        usleep(8 * 1000);

      std::string init_pose_path = ros::package::getPath("alice_base_module") + "/data/ini_pose_"+alice_id_+".yaml";
      parsePoseData(init_pose_path, "base");
 
      tra_gene_tread_ = boost::thread(boost::bind(&BaseModule::initPoseTrajGenerateProc, this));
    }
    
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void BaseModule::initPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    //joint_state_->goal_joint_state_[id].position_ =joint_state_->curr_joint_state_[id].position_;
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = base_module_state_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra;

      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  base_module_state_->smp_time_, base_module_state_->mov_time_);
 

    base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
  }

  base_module_state_->is_moving_ = true;
  base_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}



bool BaseModule::isRunning()
{
  return base_module_state_->is_moving_;
}

void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

 /*----- write goal torque -----*/

  if(alice_id_ == "3")
  {
    result_["l_shoulder_pitch"]->goal_torque_ = 41.1429;
    result_["r_shoulder_pitch"]->goal_torque_ = 41.1429;
    result_["l_shoulder_roll"]->goal_torque_ = 41.1429; 
    result_["r_shoulder_roll"]->goal_torque_ = 41.1429;

    result_["waist_pitch"]->goal_torque_ = 102.1467;

    result_["l_hip_pitch"]->goal_torque_ = 102.1467;
    result_["l_hip_roll"]->goal_torque_ = 102.1467;

    result_["l_knee_pitch"]->goal_torque_ = 102.1467;
    result_["l_ankle_pitch"]->goal_torque_ = 102.1467;
    result_["l_ankle_roll"]->goal_torque_ = 102.1467;

    result_["r_hip_pitch"]->goal_torque_ = 102.1467;    
    result_["r_hip_roll"]->goal_torque_ = 102.1467;      
    
    result_["r_knee_pitch"]->goal_torque_ = 102.1467; 
    result_["r_ankle_pitch"]->goal_torque_ = 102.1467;
    result_["r_ankle_roll"]->goal_torque_ = 102.1467;
  }
  else if(alice_id_ == "2")
  {
    result_["l_hip_pitch"]->goal_torque_ = 102.1467;
    result_["l_hip_roll"]->goal_torque_ = 102.1467;

    result_["l_knee_pitch"]->goal_torque_ = 102.1467;
    result_["l_ankle_pitch"]->goal_torque_ = 102.1467;
    result_["l_ankle_roll"]->goal_torque_ = 102.1467;

    result_["r_hip_pitch"]->goal_torque_ = 102.1467;    
    result_["r_hip_roll"]->goal_torque_ = 102.1467;      
    
    result_["r_knee_pitch"]->goal_torque_ = 102.1467; 
    result_["r_ankle_pitch"]->goal_torque_ = 102.1467;
    result_["r_ankle_roll"]->goal_torque_ = 102.1467;
  }

  /*----- write curr position -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;

  }

  has_goal_joints_ = true;


  /* ----- send trajectory ----- */
  if (base_module_state_->is_moving_ == true)
  {


    if (base_module_state_->cnt_ == 1)
    {
      ROS_INFO("Start Base Module Pose");

      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Base Module Pose");
    }
    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ = base_module_state_->calc_joint_tra_(base_module_state_->cnt_, id);

    base_module_state_->cnt_++;
  }

  /*----- set joint data -----*/

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {

    //ROS_INFO("Set Joint Data");
    std::string joint_name = state_iter->first;

    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;
  }

  /*---------- initialize count number ----------*/

  if ((base_module_state_->cnt_ >= base_module_state_->all_time_steps_) && (base_module_state_->is_moving_ == true))
  {

    ROS_INFO("[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Base Module Pose");
    publishDoneMsg("base_pose");

    base_module_state_->is_moving_ = false;
    base_module_state_->cnt_ = 0;

    // set all joints -> none
    if (ini_pose_only_ == true)
    {
      setCtrlModule("none");
      ini_pose_only_ = false;
    }
  }
}


void BaseModule::stop()
{

  return;
}

void BaseModule::setCtrlModule(std::string module)
{
  std_msgs::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_.publish(control_msg);
}

void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Base";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}

void BaseModule::publishDoneMsg(const std::string done_msg)
{
  std_msgs::String movement_msg;
  movement_msg.data = done_msg;
  movement_done_pub_.publish(movement_msg);
}

