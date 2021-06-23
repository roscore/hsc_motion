/*
 * alice_upper_body_module.cpp
 *
 *  Created on: May 30, 2018
 *      Author: robotemperor
 */
#include "alice_upper_body_module/alice_upper_body_module.h"

using namespace alice;

UpperBodyModule::UpperBodyModule()
: control_cycle_msec_(8)
{
  running_ = false;
  gazebo_check = false;
  is_moving_state = false;
  enable_       = false;
  module_name_  = "upper_body_module";
  control_mode_ = robotis_framework::PositionControl;
  readIDData();

  // Dynamixel initialize ////
  if(alice_id_ == "1")
  {
    result_["waist_yaw"] = new robotis_framework::DynamixelState(); // joint 9
  }
  else if(alice_id_ == "3")
  {
    result_["waist_yaw"] = new robotis_framework::DynamixelState();     // joint 9
    result_["waist_pitch"] = new robotis_framework::DynamixelState();   // joint 10
    ;
  }
  result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
  result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
  result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
  result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
  result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
  result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6

  ///////////////////////////
  current_time_arm_motion = 0;
  motion_num_arm= 1;
  flag_ = 1;
  is_moving_state = false;
  mov_time_state = 0;
  
  arm_motion_run = false;

  joint_name_to_ini_pose_state_.clear();
}

UpperBodyModule::~UpperBodyModule()
{
  queue_thread_.join();
}

void UpperBodyModule::readIDData()
{
  ros::NodeHandle nh;
  int alice_id_int  = nh.param<int>("alice_userid",0);

  //ROS_INFO("Base id: %d",alice_id_int);
  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  alice_id_ = alice_id_stream.str();
}

// ros message communication thread
void UpperBodyModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;
  ros_node.setCallbackQueue(&callback_queue);

  // subscribe topics
  arm_moving_sub = ros_node.subscribe("/heroehs/alice/arm_command", 5, &UpperBodyModule::armMovingMsgCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void UpperBodyModule::armMovingMsgCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg)
{
  if(msg->key == "arm_motion") //arm
  {
    if(msg->value == "1") //on
    {
      arm_motion_run = true;
    }
    else if(msg->value == "0") //off
    {
      arm_motion_run = false;
    }
  }
  is_moving_state = true;
}

void UpperBodyModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&UpperBodyModule::queueThread, this));
  int i =1;

  for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
      it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

    joint_name_to_id_[joint_name] = dxl_info->id_;
    joint_id_to_name_[dxl_info->id_] = joint_name;
    motion_trajectory[joint_name] = new heroehs_math::FifthOrderTrajectory();
  }

  std::string arm_path = ros::package::getPath("alice_upper_body_module") + "/data/upper_body_arm_"+alice_id_+".yaml";
  parse_motion_data(arm_path);

  ROS_INFO("< -------  Initialize Module : Upper Body Module !!  ------->");
}

bool UpperBodyModule::isRunning()
{
  return running_;
}
void UpperBodyModule::stop()
{
  flag_ = 1;
  is_moving_state = false;
  arm_motion_run = false;
  motion_num_arm = 1;
  return;
}

void UpperBodyModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
    std::map<std::string, double> sensors)
{
  if (enable_ == false)
  {
    return;
  }
   /*----- write goal torque -----*/
  if(alice_id_ == "3")
  {
    result_["l_shoulder_pitch"]->goal_torque_ = 41.1429;
    result_["r_shoulder_pitch"]->goal_torque_ = 41.1429;
    result_["l_shoulder_roll"]->goal_torque_ = 41.1429;
    result_["r_shoulder_roll"]->goal_torque_ = 41.1429;
  }

  switch (flag_)
  {
    case 1:   //// read current position flag ////
      flag_ ++;
      for(int i = 1; i<=6; i++)
      {
        result_[joint_id_to_name_[i]]->goal_position_ =  dxls[joint_id_to_name_[i]]->dxl_state_->present_position_; //디이나믹셀에서 읽어옴
        joint_name_to_ini_pose_state_[joint_id_to_name_[i]] = dxls[joint_id_to_name_[i]]->dxl_state_->present_position_; // 초기위치 저장
        motion_trajectory[joint_id_to_name_[i]]->current_pose = joint_name_to_ini_pose_state_[joint_id_to_name_[i]];
        motion_trajectory[joint_id_to_name_[i]]->current_time = 0;
      }
      break;

    case 2:   //// set goal position flag ////
      if(alice_id_ == "2" || alice_id_ == "3")
      {
        if(arm_motion_run)
        {
          if(motion_num_arm >= motion_numb_to_joint_pose_data_.size()+1)
          {
            motion_num_arm = 1;
          }
          mov_time_state = motion_time_data_[motion_num_arm-1];
          for(int i = 1; i<=6; i++)
          {
            joint_name_to_ini_pose_goal_[joint_id_to_name_[i]] = motion_numb_to_joint_pose_data_[motion_num_arm][i-1]*DEGREE2RADIAN;
          }
          flag_ ++;
          is_moving_state = true;
        }
      }
      break;

    case 3:   //// use heroehs math trajectory to send flag ////
      if(is_moving_state == false)
      {
        flag_ = 1; motion_num_arm++;
      }
      else
      {
        for(int i = 1; i<=6; i++)
        {
          result_[joint_id_to_name_[i]]->goal_position_ =  motion_trajectory[joint_id_to_name_[i]]->fifth_order_traj_gen(joint_name_to_ini_pose_state_[joint_id_to_name_[i]],
              joint_name_to_ini_pose_goal_[joint_id_to_name_[i]],0,0,0,0,0,mov_time_state);
          is_moving_state = motion_trajectory[joint_id_to_name_[i]]->is_moving_traj;
        }
      }
      break;
  }
}

void UpperBodyModule::parse_motion_data(const std::string &path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());

  }catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file!");
    return;
  }
  motion_joint_data_ = doc["link"].as<std::vector<std::string> >();
  motion_time_data_ = doc["motion_time"].as<std::vector<double> >();
  YAML::Node motion_pose_node = doc["motion"];
  for(YAML::iterator it = motion_pose_node.begin(); it != motion_pose_node.end(); ++it)
  {
    int motion_numb = it->first.as<int>();

    motion_numb_to_joint_pose_data_[motion_numb]=motion_pose_node[motion_numb].as<std::vector<double> >();
  }
}