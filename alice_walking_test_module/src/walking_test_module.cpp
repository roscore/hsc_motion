#include "alice_walking_test_module/walking_test_module.h"


using namespace alice;

class WalkingStatusMSG
{
public:
  static const std::string FAILED_TO_ADD_STEP_DATA_MSG;
  static const std::string BALANCE_PARAM_SETTING_STARTED_MSG;
  static const std::string BALANCE_PARAM_SETTING_FINISHED_MSG;
  static const std::string JOINT_FEEDBACK_GAIN_UPDATE_STARTED_MSG;
  static const std::string JOINT_FEEDBACK_GAIN_UPDATE_FINISHED_MSG;
  static const std::string WALKING_MODULE_IS_ENABLED_MSG;
  static const std::string WALKING_MODULE_IS_DISABLED_MSG;
  static const std::string BALANCE_HAS_BEEN_TURNED_OFF;
  static const std::string WALKING_START_MSG;
  static const std::string WALKING_FINISH_MSG;
};

const std::string WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG = "Failed_to_add_Step_Data";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_STARTED_MSG = "Balance_Param_Setting_Started";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISHED_MSG = "Balance_Param_Setting_Finished";
const std::string WalkingStatusMSG::JOINT_FEEDBACK_GAIN_UPDATE_STARTED_MSG = "Joint_FeedBack_Gain_Update_Started";
const std::string WalkingStatusMSG::JOINT_FEEDBACK_GAIN_UPDATE_FINISHED_MSG = "Joint_FeedBack_Gain_Update_Finished";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG = "Walking_Module_is_enabled";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG = "Walking_Module_is_disabled";
const std::string WalkingStatusMSG::BALANCE_HAS_BEEN_TURNED_OFF = "Balance_has_been_turned_off";
const std::string WalkingStatusMSG::WALKING_START_MSG = "Walking_Started";
const std::string WalkingStatusMSG::WALKING_FINISH_MSG = "Walking_Finished";

WalkingModule::WalkingModule()
: control_cycle_msec_(8)
{
  gazebo_          = false;
  enable_          = false;
  module_name_     = "walking_module";
  control_mode_    = robotis_framework::PositionControl;

  readIDData();

  result_["r_hip_pitch"  ] = new robotis_framework::DynamixelState();
  result_["r_hip_roll"   ] = new robotis_framework::DynamixelState();
  result_["r_hip_yaw"    ] = new robotis_framework::DynamixelState();
  result_["r_knee_pitch" ] = new robotis_framework::DynamixelState();
  result_["r_ankle_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ankle_roll" ] = new robotis_framework::DynamixelState();

  result_["l_hip_pitch"  ] = new robotis_framework::DynamixelState();
  result_["l_hip_roll"   ] = new robotis_framework::DynamixelState();
  result_["l_hip_yaw"    ] = new robotis_framework::DynamixelState();
  result_["l_knee_pitch" ] = new robotis_framework::DynamixelState();
  result_["l_ankle_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ankle_roll" ] = new robotis_framework::DynamixelState();

  joint_name_to_index_["r_hip_pitch"  ] = 0;
  joint_name_to_index_["r_hip_roll"   ] = 1;
  joint_name_to_index_["r_hip_yaw"    ] = 2;
  joint_name_to_index_["r_knee_pitch" ] = 3;
  joint_name_to_index_["r_ankle_pitch"] = 4;
  joint_name_to_index_["r_ankle_roll" ] = 5;

  joint_name_to_index_["l_hip_pitch"  ] = 6;
  joint_name_to_index_["l_hip_roll"   ] = 7;
  joint_name_to_index_["l_hip_yaw"    ] = 8;
  joint_name_to_index_["l_knee_pitch" ] = 9;
  joint_name_to_index_["l_ankle_pitch"] = 10;
  joint_name_to_index_["l_ankle_roll" ] = 11;

  previous_running_    = present_running    = false;

  balance_update_with_loop_ = false;
  balance_update_duration_ = 2.0;
  balance_update_sys_time_ = 2.0;
  balance_update_tra_.changeTrajectory(0, 0, 0, 0, balance_update_duration_, 1, 0, 0);

  joint_feedback_update_with_loop_ = false;
  joint_feedback_update_duration_ = 2.0;
  joint_feedback_update_sys_time_ = 2.0;
  joint_feedback_update_tra_.changeTrajectory(0, 0, 0, 0, balance_update_duration_, 1, 0, 0);



  readKinematicsYamlData();
}

WalkingModule::~WalkingModule()
{
  queue_thread_.join();
}

void WalkingModule::readIDData()
{
  ros::NodeHandle nh;
  int alice_id_int  = nh.param<int>("alice_userid",0);

  ROS_INFO("Walking id: %d",alice_id_int);
  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  alice_id_ = alice_id_stream.str();
}

void WalkingModule::readKinematicsYamlData()
{
  ros::NodeHandle nh;
  int alice_id_int  = nh.param<int>("alice_userid",0);

  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  std::string alice_id = alice_id_stream.str();

  std::string kinematics_path = ros::package::getPath("alice_kinematics_dynamics")+"/data/kin_dyn_"+alice_id+".yaml";
  YAML::Node kinematics_doc;
  try
  {
    kinematics_doc = YAML::LoadFile(kinematics_path.c_str());

  }catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load kinematics yaml file!");
    return;
  }
  leg_to_end_ = kinematics_doc["l_leg_end"]["relative_position"][2].as<double>();

}

void WalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&WalkingModule::queueThread, this));
  control_cycle_msec_ = control_cycle_msec;
  ALICEWalking* alice_walking = ALICEWalking::getInstance();

  alice_walking->initialize(control_cycle_msec*0.001);
  alice_walking->start();

  process_mutex_.lock();
  alice_walking->process();
  process_mutex_.unlock();

  result_["r_hip_pitch"  ]->goal_position_ = alice_walking->out_angle_rad_[0];
  result_["r_hip_roll"   ]->goal_position_ = alice_walking->out_angle_rad_[1];
  result_["r_hip_yaw"    ]->goal_position_ = alice_walking->out_angle_rad_[2];
  result_["r_knee_pitch" ]->goal_position_  = alice_walking->out_angle_rad_[3];
  result_["r_ankle_pitch"]->goal_position_  = alice_walking->out_angle_rad_[4];
  result_["r_ankle_roll" ]->goal_position_  = alice_walking->out_angle_rad_[5];

  result_["l_hip_pitch"  ]->goal_position_ = alice_walking->out_angle_rad_[6];
  result_["l_hip_roll"   ]->goal_position_ = alice_walking->out_angle_rad_[7];
  result_["l_hip_yaw"    ]->goal_position_ = alice_walking->out_angle_rad_[8];
  result_["l_knee_pitch" ]->goal_position_ = alice_walking->out_angle_rad_[9];
  result_["l_ankle_pitch"]->goal_position_ = alice_walking->out_angle_rad_[10];
  result_["l_ankle_roll" ]->goal_position_ = alice_walking->out_angle_rad_[11];



  previous_running_ = isRunning();

  alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_ = 0;
  alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_ = 0;
  alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_ = 0;
  alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_ = 0;

  alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_ = 0;
  alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_ = 0;
  alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_ = 0;
  alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_ = 0;

  alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_ = 0;
  alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_ = 0;
  alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_ = 0;
  alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_ = 0;
  alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_ = 0;
  alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_ = 0;
  alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_ = 0;
  alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_ = 0;
  alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_ = 0;
  alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_ = 0;

}


void WalkingModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("heroehs/status", 10);
  done_msg_pub_ = ros_node.advertise<std_msgs::String>("/heroehs/movement_done", 10);

  //GUI
  reference_zmp_pub_ = ros_node.advertise<geometry_msgs::Point>("/heroehs/alice_reference_zmp", 10);
  reference_body_pub_ = ros_node.advertise<geometry_msgs::Twist>("/heroehs/alice_reference_body", 10);
  reference_body_delta_pub_ = ros_node.advertise<geometry_msgs::Twist>("/alice/ideal_body_delta", 10);

  ft_calc_zmp_pub_ = ros_node.advertise<geometry_msgs::Point>("/heroehs/alice_measured_zmp", 10);

  foot_right_pub_ = ros_node.advertise<geometry_msgs::Point>("/heroehs/alice_right_foot_pose", 10);
  foot_left_pub_  = ros_node.advertise<geometry_msgs::Point>("/heroehs/alice_left_foot_pose", 10);

  angle_sensor_pub_     = ros_node.advertise<geometry_msgs::Vector3>("/heroehs/alice_angle_imu", 10);
  angle_acc_sensor_pub_ = ros_node.advertise<geometry_msgs::Vector3>("/heroehs/alice_angle_acc_imu", 10);

  /* ROS Service Callback Functions */
  ros::ServiceServer get_ref_step_data_server  = ros_node.advertiseService("/heroehs/online_walking/get_reference_step_data",   &WalkingModule::getReferenceStepDataServiceCallback,   this);
  ros::ServiceServer add_step_data_array_sever = ros_node.advertiseService("/heroehs/online_walking/add_step_data",             &WalkingModule::addStepDataServiceCallback,            this);
  ros::ServiceServer walking_start_server      = ros_node.advertiseService("/heroehs/online_walking/walking_start",             &WalkingModule::startWalkingServiceCallback,           this);
  ros::ServiceServer is_running_server         = ros_node.advertiseService("/heroehs/online_walking/is_running",                &WalkingModule::IsRunningServiceCallback,              this);
  ros::ServiceServer set_balance_param_server  = ros_node.advertiseService("/heroehs/online_walking/set_balance_param",         &WalkingModule::setBalanceParamServiceCallback,        this);
  ros::ServiceServer set_joint_feedback_gain   = ros_node.advertiseService("/heroehs/online_walking/joint_feedback_gain",       &WalkingModule::setJointFeedBackGainServiceCallback,   this);
  ros::ServiceServer remove_existing_step_data = ros_node.advertiseService("/heroehs/online_walking/remove_existing_step_data", &WalkingModule::removeExistingStepDataServiceCallback, this);

  /* sensor topic subscribe */
  ros::Subscriber imu_data_sub = ros_node.subscribe("/imu/data", 10, &WalkingModule::imuDataOutputCallback,        this);
  ros::Subscriber ft_data_sub  = ros_node.subscribe("/alice/force_torque_data", 10, &WalkingModule::ftDataOutputCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  if(ros::param::get("gazebo", gazebo_) == false)
    gazebo_ = false;

  while(ros_node.ok())
    callback_queue.callAvailable(duration);

}

void WalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

void WalkingModule::publishDoneMsg(std::string msg)
{
  std_msgs::String done_msg;
  done_msg.data = msg;

  done_msg_pub_.publish(done_msg);
}

int WalkingModule::convertStepDataMsgToStepData(alice_walking_module_msgs::StepData& src, robotis_framework::StepData& des)
{
  int copy_result = alice_walking_module_msgs::AddStepDataArray::Response::NO_ERROR;
  des.time_data.   walking_state        = src.time_data.walking_state;
  des.time_data.abs_step_time           = src.time_data.abs_step_time;
  des.time_data.dsp_ratio               = src.time_data.dsp_ratio;

  des.position_data.moving_foot         = src.position_data.moving_foot;
  des.position_data.shoulder_swing_gain = 0;
  des.position_data.elbow_swing_gain    = 0;

  des.position_data.x_zmp_shift         = src.position_data.x_zmp_shift;
  des.position_data.y_zmp_shift         = src.position_data.y_zmp_shift;

  des.position_data.foot_z_swap         = src.position_data.foot_z_swap;
  des.position_data.waist_pitch_angle   = 0;
  des.position_data.waist_yaw_angle     = src.position_data.torso_yaw_angle_rad;
  des.position_data.body_z_swap         = src.position_data.body_z_swap;

  des.position_data.body_pose.z          = src.position_data.body_pose.z;
  des.position_data.body_pose.roll       = src.position_data.body_pose.roll;
  des.position_data.body_pose.pitch      = src.position_data.body_pose.pitch;
  des.position_data.body_pose.yaw        = src.position_data.body_pose.yaw;
  des.position_data.right_foot_pose.x     = src.position_data.right_foot_pose.x;
  des.position_data.right_foot_pose.y     = src.position_data.right_foot_pose.y;
  des.position_data.right_foot_pose.z     = src.position_data.right_foot_pose.z;
  des.position_data.right_foot_pose.roll  = src.position_data.right_foot_pose.roll;
  des.position_data.right_foot_pose.pitch = src.position_data.right_foot_pose.pitch;
  des.position_data.right_foot_pose.yaw   = src.position_data.right_foot_pose.yaw;
  des.position_data.left_foot_pose.x      = src.position_data.left_foot_pose.x;
  des.position_data.left_foot_pose.y      = src.position_data.left_foot_pose.y;
  des.position_data.left_foot_pose.z      = src.position_data.left_foot_pose.z;
  des.position_data.left_foot_pose.roll   = src.position_data.left_foot_pose.roll;
  des.position_data.left_foot_pose.pitch  = src.position_data.left_foot_pose.pitch;
  des.position_data.left_foot_pose.yaw    = src.position_data.left_foot_pose.yaw;

  des.time_data.start_time_delay_ratio_x         = src.time_data.start_time_delay_ratio_x;
  des.time_data.start_time_delay_ratio_y         = src.time_data.start_time_delay_ratio_y;
  des.time_data.start_time_delay_ratio_z         = src.time_data.start_time_delay_ratio_z;
  des.time_data.start_time_delay_ratio_roll      = src.time_data.start_time_delay_ratio_roll;
  des.time_data.start_time_delay_ratio_pitch     = src.time_data.start_time_delay_ratio_pitch;
  des.time_data.start_time_delay_ratio_yaw       = src.time_data.start_time_delay_ratio_yaw;

  des.time_data.finish_time_advance_ratio_x     = src.time_data.finish_time_advance_ratio_x;
  des.time_data.finish_time_advance_ratio_y     = src.time_data.finish_time_advance_ratio_y;
  des.time_data.finish_time_advance_ratio_z     = src.time_data.finish_time_advance_ratio_z;
  des.time_data.finish_time_advance_ratio_roll  = src.time_data.finish_time_advance_ratio_roll;
  des.time_data.finish_time_advance_ratio_pitch = src.time_data.finish_time_advance_ratio_pitch;
  des.time_data.finish_time_advance_ratio_yaw   = src.time_data.finish_time_advance_ratio_yaw;

  if((src.time_data.walking_state != alice_walking_module_msgs::StepTimeData::IN_WALKING_STARTING)
      && (src.time_data.walking_state != alice_walking_module_msgs::StepTimeData::IN_WALKING)
      && (src.time_data.walking_state != alice_walking_module_msgs::StepTimeData::IN_WALKING_ENDING) )
    copy_result |= alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.start_time_delay_ratio_x     < 0)
      || (src.time_data.start_time_delay_ratio_y     < 0)
      || (src.time_data.start_time_delay_ratio_z     < 0)
      || (src.time_data.start_time_delay_ratio_roll  < 0)
      || (src.time_data.start_time_delay_ratio_pitch < 0)
      || (src.time_data.start_time_delay_ratio_yaw   < 0) )
    copy_result |= alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.finish_time_advance_ratio_x     < 0)
      || (src.time_data.finish_time_advance_ratio_y     < 0)
      || (src.time_data.finish_time_advance_ratio_z     < 0)
      || (src.time_data.finish_time_advance_ratio_roll  < 0)
      || (src.time_data.finish_time_advance_ratio_pitch < 0)
      || (src.time_data.finish_time_advance_ratio_yaw   < 0) )
    copy_result |= alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if(((src.time_data.start_time_delay_ratio_x + src.time_data.finish_time_advance_ratio_x) > 1.0)
      || ((src.time_data.start_time_delay_ratio_y      + src.time_data.finish_time_advance_ratio_y     ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_z      + src.time_data.finish_time_advance_ratio_z     ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_roll   + src.time_data.finish_time_advance_ratio_roll  ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_pitch  + src.time_data.finish_time_advance_ratio_pitch ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_yaw    + src.time_data.finish_time_advance_ratio_yaw   ) > 1.0) )
    copy_result |= alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.position_data.moving_foot != alice_walking_module_msgs::StepPositionData::STANDING)
      && (src.position_data.moving_foot != alice_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
      && (src.position_data.moving_foot != alice_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING))
    copy_result |= alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  if(src.position_data.foot_z_swap < 0)
    copy_result |= alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  return copy_result;
}

int WalkingModule::convertStepDataToStepDataMsg(robotis_framework::StepData& src, alice_walking_module_msgs::StepData& des)
{
  des.time_data.walking_state   = src.time_data.walking_state;
  des.time_data.abs_step_time   = src.time_data.abs_step_time;
  des.time_data.dsp_ratio       = src.time_data.dsp_ratio;

  des.time_data.start_time_delay_ratio_x     = des.time_data.finish_time_advance_ratio_x     = 0;
  des.time_data.start_time_delay_ratio_y     = des.time_data.finish_time_advance_ratio_y     = 0;
  des.time_data.start_time_delay_ratio_z     = des.time_data.finish_time_advance_ratio_z     = 0;
  des.time_data.start_time_delay_ratio_roll  = des.time_data.finish_time_advance_ratio_roll  = 0;
  des.time_data.start_time_delay_ratio_pitch = des.time_data.finish_time_advance_ratio_pitch = 0;
  des.time_data.start_time_delay_ratio_yaw   = des.time_data.finish_time_advance_ratio_yaw   = 0;

  des.position_data.moving_foot         = src.position_data.moving_foot;
  des.position_data.foot_z_swap         = src.position_data.foot_z_swap;
  des.position_data.torso_yaw_angle_rad = src.position_data.waist_yaw_angle;
  des.position_data.body_z_swap         = src.position_data.body_z_swap;

  des.position_data.x_zmp_shift         = src.position_data.x_zmp_shift;
  des.position_data.y_zmp_shift         = src.position_data.y_zmp_shift;

  des.position_data.body_pose.z           = src.position_data.body_pose.z;
  des.position_data.body_pose.roll        = src.position_data.body_pose.roll;
  des.position_data.body_pose.pitch       = src.position_data.body_pose.pitch;
  des.position_data.body_pose.yaw         = src.position_data.body_pose.yaw;
  des.position_data.right_foot_pose.x     = src.position_data.right_foot_pose.x;
  des.position_data.right_foot_pose.y     = src.position_data.right_foot_pose.y;
  des.position_data.right_foot_pose.z     = src.position_data.right_foot_pose.z;
  des.position_data.right_foot_pose.roll  = src.position_data.right_foot_pose.roll;
  des.position_data.right_foot_pose.pitch = src.position_data.right_foot_pose.pitch;
  des.position_data.right_foot_pose.yaw   = src.position_data.right_foot_pose.yaw;
  des.position_data.left_foot_pose.x      = src.position_data.left_foot_pose.x;
  des.position_data.left_foot_pose.y      = src.position_data.left_foot_pose.y;
  des.position_data.left_foot_pose.z      = src.position_data.left_foot_pose.z;
  des.position_data.left_foot_pose.roll   = src.position_data.left_foot_pose.roll;
  des.position_data.left_foot_pose.pitch  = src.position_data.left_foot_pose.pitch;
  des.position_data.left_foot_pose.yaw    = src.position_data.left_foot_pose.yaw;

  return 0;
}

bool WalkingModule::getReferenceStepDataServiceCallback(alice_walking_module_msgs::GetReferenceStepData::Request &req,
    alice_walking_module_msgs::GetReferenceStepData::Response &res)
{
  ALICEWalking *alice_walking = ALICEWalking::getInstance();

  robotis_framework::StepData refStepData;

  alice_walking->getReferenceStepDatafotAddition(&refStepData);

  convertStepDataToStepDataMsg(refStepData, res.reference_step_data);

  return true;
}

bool WalkingModule::addStepDataServiceCallback(alice_walking_module_msgs::AddStepDataArray::Request &req,
    alice_walking_module_msgs::AddStepDataArray::Response &res)
{
  ALICEWalking *alice_walking = ALICEWalking::getInstance();
  res.result = alice_walking_module_msgs::AddStepDataArray::Response::NO_ERROR;

  if(enable_ == false)
  {
    res.result |= alice_walking_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE;
    std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  if((req.step_data_array.size() > 100)
      && (req.remove_existing_step_data == true)
      && ((alice_walking->isRunning() == true)))
  {
    res.result |= alice_walking_module_msgs::AddStepDataArray::Response::TOO_MANY_STEP_DATA;
    std::string status_msg  = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  robotis_framework::StepData step_data, ref_step_data;
  std::vector<robotis_framework::StepData> req_step_data_array;

  alice_walking->getReferenceStepDatafotAddition(&ref_step_data);

  for(int i = 0; i < req.step_data_array.size(); i++)
  {
    res.result |= convertStepDataMsgToStepData(req.step_data_array[i], step_data);

    if(step_data.time_data.abs_step_time <= 0)
    {
      res.result |= alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
    }

    if(i != 0)
    {
      if(step_data.time_data.abs_step_time <= req_step_data_array[req_step_data_array.size() - 1].time_data.abs_step_time)
      {
        res.result |= alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }
    else
    {
      if(step_data.time_data.abs_step_time <= ref_step_data.time_data.abs_step_time)
      {
        res.result |= alice_walking_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }

    if(res.result != alice_walking_module_msgs::AddStepDataArray::Response::NO_ERROR)
    {
      std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return true;
    }

    req_step_data_array.push_back(step_data);
  }

  if(req.remove_existing_step_data == true)
  {
    int exist_num_of_step_data = alice_walking->getNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        alice_walking->eraseLastStepData();
  }
  else
  {
    if(alice_walking->isRunning() == true)
    {

      res.result |= alice_walking_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW;
      std::string status_msg  = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return true;
    }
  }

  if(checkBalanceOnOff() == false)
  {
    std::string status_msg  = WalkingStatusMSG::BALANCE_HAS_BEEN_TURNED_OFF;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  for(unsigned int i = 0; i < req_step_data_array.size() ; i++)
    alice_walking->addStepData(req_step_data_array[i]);

  if( req.auto_start == true)
  {
    alice_walking->start();
  }

  return true;
}

bool WalkingModule::startWalkingServiceCallback(alice_walking_module_msgs::StartWalking::Request &req,
    alice_walking_module_msgs::StartWalking::Response &res)
{
  ALICEWalking *prev_walking = ALICEWalking::getInstance();
  res.result = alice_walking_module_msgs::StartWalking::Response::NO_ERROR;

  if(enable_ == false)
  {
    res.result |= alice_walking_module_msgs::StartWalking::Response::NOT_ENABLED_WALKING_MODULE;
    return true;
  }

  if(prev_walking->isRunning() == true)
  {

    res.result |= alice_walking_module_msgs::StartWalking::Response::ROBOT_IS_WALKING_NOW;
    return true;
  }

  if(checkBalanceOnOff() == false)
  {
    std::string status_msg  = WalkingStatusMSG::BALANCE_HAS_BEEN_TURNED_OFF;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  if(prev_walking->getNumofRemainingUnreservedStepData() == 0)
  {
    res.result |= alice_walking_module_msgs::StartWalking::Response::NO_STEP_DATA;
    return true;
  }

  if(res.result == alice_walking_module_msgs::StartWalking::Response::NO_ERROR)
  {
    prev_walking->start();
  }

  return true;
}

bool WalkingModule::IsRunningServiceCallback(alice_walking_module_msgs::IsRunning::Request &req,
    alice_walking_module_msgs::IsRunning::Response &res)
{
  bool is_running = isRunning();
  res.is_running = is_running;

  return true;
}

bool WalkingModule::isRunning()
{
  return ALICEWalking::getInstance()->isRunning();
}

bool WalkingModule::removeExistingStepDataServiceCallback(alice_walking_module_msgs::RemoveExistingStepData::Request  &req,
    alice_walking_module_msgs::RemoveExistingStepData::Response &res)
{
  ALICEWalking *alice_walking = ALICEWalking::getInstance();

  res.result = alice_walking_module_msgs::RemoveExistingStepData::Response::NO_ERROR;

  if(isRunning())
  {
    res.result |= alice_walking_module_msgs::RemoveExistingStepData::Response::ROBOT_IS_WALKING_NOW;
  }
  else
  {
    int exist_num_of_step_data = alice_walking->getNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        alice_walking->eraseLastStepData();
  }
  return true;
}

bool WalkingModule::setJointFeedBackGainServiceCallback(alice_walking_module_msgs::SetJointFeedBackGain::Request &req,
    alice_walking_module_msgs::SetJointFeedBackGain::Response &res)
{
  ALICEWalking *alice_walking = ALICEWalking::getInstance();

  res.result = alice_walking_module_msgs::SetJointFeedBackGain::Response::NO_ERROR;

  if( enable_ == false)
    res.result |= alice_walking_module_msgs::SetJointFeedBackGain::Response::NOT_ENABLED_WALKING_MODULE;

  if( joint_feedback_update_with_loop_ == true)
    res.result |= alice_walking_module_msgs::SetJointFeedBackGain::Response::PREV_REQUEST_IS_NOT_FINISHED;

  if( res.result != alice_walking_module_msgs::SetJointFeedBackGain::Response::NO_ERROR)
  {
    publishDoneMsg("walking_joint_feedback_failed");
    return true;
  }

  if( req.updating_duration <= 0.0 )
  {
    // under 8ms apply immediately
    setJointFeedBackGain(req.feedback_gain);
    std::string status_msg = WalkingStatusMSG::JOINT_FEEDBACK_GAIN_UPDATE_FINISHED_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    publishDoneMsg("walking_joint_feedback");
    return true;
  }
  else
  {
    joint_feedback_update_duration_ = req.updating_duration;
  }

  joint_feedback_update_sys_time_ = 0.0;
  double tf = joint_feedback_update_duration_;
  joint_feedback_update_tra_.changeTrajectory(0, 0, 0, 0, joint_feedback_update_duration_, 1.0, 0, 0);

  desired_joint_feedback_gain_ = req.feedback_gain;

  previous_joint_feedback_gain_.r_leg_hip_y_p_gain = alice_walking->leg_angle_feed_back_[0].p_gain_;
  previous_joint_feedback_gain_.r_leg_hip_y_d_gain = alice_walking->leg_angle_feed_back_[0].d_gain_;
  previous_joint_feedback_gain_.r_leg_hip_r_p_gain = alice_walking->leg_angle_feed_back_[1].p_gain_;
  previous_joint_feedback_gain_.r_leg_hip_r_d_gain = alice_walking->leg_angle_feed_back_[1].d_gain_;
  previous_joint_feedback_gain_.r_leg_hip_p_p_gain = alice_walking->leg_angle_feed_back_[2].p_gain_;
  previous_joint_feedback_gain_.r_leg_hip_p_d_gain = alice_walking->leg_angle_feed_back_[2].d_gain_;
  previous_joint_feedback_gain_.r_leg_kn_p_p_gain  = alice_walking->leg_angle_feed_back_[3].p_gain_;
  previous_joint_feedback_gain_.r_leg_kn_p_d_gain  = alice_walking->leg_angle_feed_back_[3].d_gain_;
  previous_joint_feedback_gain_.r_leg_an_p_p_gain  = alice_walking->leg_angle_feed_back_[4].p_gain_;
  previous_joint_feedback_gain_.r_leg_an_p_d_gain  = alice_walking->leg_angle_feed_back_[4].d_gain_;
  previous_joint_feedback_gain_.r_leg_an_r_p_gain  = alice_walking->leg_angle_feed_back_[5].p_gain_;
  previous_joint_feedback_gain_.r_leg_an_r_d_gain  = alice_walking->leg_angle_feed_back_[5].d_gain_;

  previous_joint_feedback_gain_.l_leg_hip_y_p_gain = alice_walking->leg_angle_feed_back_[6].p_gain_;
  previous_joint_feedback_gain_.l_leg_hip_y_d_gain = alice_walking->leg_angle_feed_back_[6].d_gain_;
  previous_joint_feedback_gain_.l_leg_hip_r_p_gain = alice_walking->leg_angle_feed_back_[7].p_gain_;
  previous_joint_feedback_gain_.l_leg_hip_r_d_gain = alice_walking->leg_angle_feed_back_[7].d_gain_;
  previous_joint_feedback_gain_.l_leg_hip_p_p_gain = alice_walking->leg_angle_feed_back_[8].p_gain_;
  previous_joint_feedback_gain_.l_leg_hip_p_d_gain = alice_walking->leg_angle_feed_back_[8].d_gain_;
  previous_joint_feedback_gain_.l_leg_kn_p_p_gain  = alice_walking->leg_angle_feed_back_[9].p_gain_;
  previous_joint_feedback_gain_.l_leg_kn_p_d_gain  = alice_walking->leg_angle_feed_back_[9].d_gain_;
  previous_joint_feedback_gain_.l_leg_an_p_p_gain  = alice_walking->leg_angle_feed_back_[10].p_gain_;
  previous_joint_feedback_gain_.l_leg_an_p_d_gain  = alice_walking->leg_angle_feed_back_[10].d_gain_;
  previous_joint_feedback_gain_.l_leg_an_r_p_gain  = alice_walking->leg_angle_feed_back_[11].p_gain_;
  previous_joint_feedback_gain_.l_leg_an_r_d_gain  = alice_walking->leg_angle_feed_back_[11].d_gain_;

  joint_feedback_update_with_loop_ = true;
  joint_feedback_update_sys_time_  = 0.0;

  return true;
}

void WalkingModule::setJointFeedBackGain(alice_walking_module_msgs::JointFeedBackGain& msg)
{
  ALICEWalking *alice_walking = ALICEWalking::getInstance();
  alice_walking->leg_angle_feed_back_[0].p_gain_  = msg.r_leg_hip_y_p_gain ;
  alice_walking->leg_angle_feed_back_[0].d_gain_  = msg.r_leg_hip_y_d_gain ;
  alice_walking->leg_angle_feed_back_[1].p_gain_  = msg.r_leg_hip_r_p_gain ;
  alice_walking->leg_angle_feed_back_[1].d_gain_  = msg.r_leg_hip_r_d_gain ;
  alice_walking->leg_angle_feed_back_[2].p_gain_  = msg.r_leg_hip_p_p_gain ;
  alice_walking->leg_angle_feed_back_[2].d_gain_  = msg.r_leg_hip_p_d_gain ;
  alice_walking->leg_angle_feed_back_[3].p_gain_  = msg.r_leg_kn_p_p_gain  ;
  alice_walking->leg_angle_feed_back_[3].d_gain_  = msg.r_leg_kn_p_d_gain  ;
  alice_walking->leg_angle_feed_back_[4].p_gain_  = msg.r_leg_an_p_p_gain  ;
  alice_walking->leg_angle_feed_back_[4].d_gain_  = msg.r_leg_an_p_d_gain  ;
  alice_walking->leg_angle_feed_back_[5].p_gain_  = msg.r_leg_an_r_p_gain  ;
  alice_walking->leg_angle_feed_back_[5].d_gain_  = msg.r_leg_an_r_d_gain  ;

  alice_walking->leg_angle_feed_back_[6].p_gain_  = msg.l_leg_hip_y_p_gain ;
  alice_walking->leg_angle_feed_back_[6].d_gain_  = msg.l_leg_hip_y_d_gain ;
  alice_walking->leg_angle_feed_back_[7].p_gain_  = msg.l_leg_hip_r_p_gain ;
  alice_walking->leg_angle_feed_back_[7].d_gain_  = msg.l_leg_hip_r_d_gain ;
  alice_walking->leg_angle_feed_back_[8].p_gain_  = msg.l_leg_hip_p_p_gain ;
  alice_walking->leg_angle_feed_back_[8].d_gain_  = msg.l_leg_hip_p_d_gain ;
  alice_walking->leg_angle_feed_back_[9].p_gain_  = msg.l_leg_kn_p_p_gain  ;
  alice_walking->leg_angle_feed_back_[9].d_gain_  = msg.l_leg_kn_p_d_gain  ;
  alice_walking->leg_angle_feed_back_[10].p_gain_ = msg.l_leg_an_p_p_gain  ;
  alice_walking->leg_angle_feed_back_[10].d_gain_ = msg.l_leg_an_p_d_gain  ;
  alice_walking->leg_angle_feed_back_[11].p_gain_ = msg.l_leg_an_r_p_gain  ;
  alice_walking->leg_angle_feed_back_[11].d_gain_ = msg.l_leg_an_r_d_gain  ;
}


void WalkingModule::updateJointFeedBackGain()
{
  double current_update_gain =  joint_feedback_update_tra_.getPosition(joint_feedback_update_sys_time_);


  current_joint_feedback_gain_.r_leg_hip_y_p_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_y_p_gain - previous_joint_feedback_gain_.r_leg_hip_y_p_gain ) + previous_joint_feedback_gain_.r_leg_hip_y_p_gain  ;
  current_joint_feedback_gain_.r_leg_hip_y_d_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_y_d_gain - previous_joint_feedback_gain_.r_leg_hip_y_d_gain ) + previous_joint_feedback_gain_.r_leg_hip_y_d_gain  ;
  current_joint_feedback_gain_.r_leg_hip_r_p_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_r_p_gain - previous_joint_feedback_gain_.r_leg_hip_r_p_gain ) + previous_joint_feedback_gain_.r_leg_hip_r_p_gain  ;
  current_joint_feedback_gain_.r_leg_hip_r_d_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_r_d_gain - previous_joint_feedback_gain_.r_leg_hip_r_d_gain ) + previous_joint_feedback_gain_.r_leg_hip_r_d_gain  ;
  current_joint_feedback_gain_.r_leg_hip_p_p_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_p_p_gain - previous_joint_feedback_gain_.r_leg_hip_p_p_gain ) + previous_joint_feedback_gain_.r_leg_hip_p_p_gain  ;
  current_joint_feedback_gain_.r_leg_hip_p_d_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_p_d_gain - previous_joint_feedback_gain_.r_leg_hip_p_d_gain ) + previous_joint_feedback_gain_.r_leg_hip_p_d_gain  ;
  current_joint_feedback_gain_.r_leg_kn_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_kn_p_p_gain  - previous_joint_feedback_gain_.r_leg_kn_p_p_gain  ) + previous_joint_feedback_gain_.r_leg_kn_p_p_gain   ;
  current_joint_feedback_gain_.r_leg_kn_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_kn_p_d_gain  - previous_joint_feedback_gain_.r_leg_kn_p_d_gain  ) + previous_joint_feedback_gain_.r_leg_kn_p_d_gain   ;
  current_joint_feedback_gain_.r_leg_an_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_p_p_gain  - previous_joint_feedback_gain_.r_leg_an_p_p_gain  ) + previous_joint_feedback_gain_.r_leg_an_p_p_gain   ;
  current_joint_feedback_gain_.r_leg_an_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_p_d_gain  - previous_joint_feedback_gain_.r_leg_an_p_d_gain  ) + previous_joint_feedback_gain_.r_leg_an_p_d_gain   ;
  current_joint_feedback_gain_.r_leg_an_r_p_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_r_p_gain  - previous_joint_feedback_gain_.r_leg_an_r_p_gain  ) + previous_joint_feedback_gain_.r_leg_an_r_p_gain   ;
  current_joint_feedback_gain_.r_leg_an_r_d_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_r_d_gain  - previous_joint_feedback_gain_.r_leg_an_r_d_gain  ) + previous_joint_feedback_gain_.r_leg_an_r_d_gain   ;

  current_joint_feedback_gain_.l_leg_hip_y_p_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_y_p_gain - previous_joint_feedback_gain_.l_leg_hip_y_p_gain ) + previous_joint_feedback_gain_.l_leg_hip_y_p_gain  ;
  current_joint_feedback_gain_.l_leg_hip_y_d_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_y_d_gain - previous_joint_feedback_gain_.l_leg_hip_y_d_gain ) + previous_joint_feedback_gain_.l_leg_hip_y_d_gain  ;
  current_joint_feedback_gain_.l_leg_hip_r_p_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_r_p_gain - previous_joint_feedback_gain_.l_leg_hip_r_p_gain ) + previous_joint_feedback_gain_.l_leg_hip_r_p_gain  ;
  current_joint_feedback_gain_.l_leg_hip_r_d_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_r_d_gain - previous_joint_feedback_gain_.l_leg_hip_r_d_gain ) + previous_joint_feedback_gain_.l_leg_hip_r_d_gain  ;
  current_joint_feedback_gain_.l_leg_hip_p_p_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_p_p_gain - previous_joint_feedback_gain_.l_leg_hip_p_p_gain ) + previous_joint_feedback_gain_.l_leg_hip_p_p_gain  ;
  current_joint_feedback_gain_.l_leg_hip_p_d_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_p_d_gain - previous_joint_feedback_gain_.l_leg_hip_p_d_gain ) + previous_joint_feedback_gain_.l_leg_hip_p_d_gain  ;
  current_joint_feedback_gain_.l_leg_kn_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_kn_p_p_gain  - previous_joint_feedback_gain_.l_leg_kn_p_p_gain  ) + previous_joint_feedback_gain_.l_leg_kn_p_p_gain   ;
  current_joint_feedback_gain_.l_leg_kn_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_kn_p_d_gain  - previous_joint_feedback_gain_.l_leg_kn_p_d_gain  ) + previous_joint_feedback_gain_.l_leg_kn_p_d_gain   ;
  current_joint_feedback_gain_.l_leg_an_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_p_p_gain  - previous_joint_feedback_gain_.l_leg_an_p_p_gain  ) + previous_joint_feedback_gain_.l_leg_an_p_p_gain   ;
  current_joint_feedback_gain_.l_leg_an_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_p_d_gain  - previous_joint_feedback_gain_.l_leg_an_p_d_gain  ) + previous_joint_feedback_gain_.l_leg_an_p_d_gain   ;
  current_joint_feedback_gain_.l_leg_an_r_p_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_r_p_gain  - previous_joint_feedback_gain_.l_leg_an_r_p_gain  ) + previous_joint_feedback_gain_.l_leg_an_r_p_gain   ;
  current_joint_feedback_gain_.l_leg_an_r_d_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_r_d_gain  - previous_joint_feedback_gain_.l_leg_an_r_d_gain  ) + previous_joint_feedback_gain_.l_leg_an_r_d_gain   ;


  setJointFeedBackGain(current_joint_feedback_gain_);
}

bool WalkingModule::setBalanceParamServiceCallback(alice_walking_module_msgs::SetBalanceParam::Request  &req,
    alice_walking_module_msgs::SetBalanceParam::Response &res)
{
  ALICEWalking *alice_walking = ALICEWalking::getInstance();
  res.result = alice_walking_module_msgs::SetBalanceParam::Response::NO_ERROR;

  if( enable_ == false)
    res.result |= alice_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE;

  if( balance_update_with_loop_ == true)
  {
    res.result |= alice_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED;
  }

  if( (req.balance_param.roll_gyro_cut_off_frequency         <= 0) ||
      (req.balance_param.pitch_gyro_cut_off_frequency        <= 0) ||
      (req.balance_param.roll_angle_cut_off_frequency        <= 0) ||
      (req.balance_param.pitch_angle_cut_off_frequency       <= 0) ||
      (req.balance_param.foot_x_force_cut_off_frequency      <= 0) ||
      (req.balance_param.foot_y_force_cut_off_frequency      <= 0) ||
      (req.balance_param.foot_z_force_cut_off_frequency      <= 0) ||
      (req.balance_param.foot_roll_torque_cut_off_frequency  <= 0) ||
      (req.balance_param.foot_pitch_torque_cut_off_frequency <= 0) )
  {
    //res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::CUT_OFF_FREQUENCY_IS_ZERO_OR_NEGATIVE;
    previous_balance_param_.roll_gyro_cut_off_frequency         = req.balance_param.roll_gyro_cut_off_frequency;
    previous_balance_param_.pitch_gyro_cut_off_frequency        = req.balance_param.pitch_gyro_cut_off_frequency;
    previous_balance_param_.roll_angle_cut_off_frequency        = req.balance_param.roll_angle_cut_off_frequency;
    previous_balance_param_.pitch_angle_cut_off_frequency       = req.balance_param.pitch_angle_cut_off_frequency;
    previous_balance_param_.foot_x_force_cut_off_frequency      = req.balance_param.foot_x_force_cut_off_frequency;
    previous_balance_param_.foot_y_force_cut_off_frequency      = req.balance_param.foot_y_force_cut_off_frequency;
    previous_balance_param_.foot_z_force_cut_off_frequency      = req.balance_param.foot_z_force_cut_off_frequency;
    previous_balance_param_.foot_roll_torque_cut_off_frequency  = req.balance_param.foot_roll_torque_cut_off_frequency;
    previous_balance_param_.foot_pitch_torque_cut_off_frequency = req.balance_param.foot_pitch_torque_cut_off_frequency;
  }

  if(res.result != alice_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
  {
    publishDoneMsg("walking_balance_failed");
    return true;
  }

  if( req.updating_duration <= 0.0 )
  {
    // under 8ms apply immediately
    setBalanceParam(req.balance_param);
    std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISHED_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    publishDoneMsg("walking_balance");
    return true;
  }
  else
  {
    balance_update_duration_ = req.updating_duration;
  }

  balance_update_sys_time_ = 0.0;
  balance_update_tra_.changeTrajectory(0, 0, 0, 0, balance_update_duration_, 1.0, 0, 0);


  desired_balance_param_ = req.balance_param;

  previous_balance_param_.cob_x_offset_m                  = alice_walking->balance_ctrl_.getCOBManualAdjustmentX();
  previous_balance_param_.cob_y_offset_m                  = alice_walking->balance_ctrl_.getCOBManualAdjustmentY();

  ////gain
  //gyro
  previous_balance_param_.foot_roll_gyro_p_gain           = alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_;
  previous_balance_param_.foot_roll_gyro_d_gain           = alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_;
  previous_balance_param_.foot_pitch_gyro_p_gain          = alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_;
  previous_balance_param_.foot_pitch_gyro_d_gain          = alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_;

  //orientation
  previous_balance_param_.foot_roll_angle_p_gain          = alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_;
  previous_balance_param_.foot_roll_angle_d_gain          = alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_;
  previous_balance_param_.foot_pitch_angle_p_gain         = alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_;
  previous_balance_param_.foot_pitch_angle_d_gain         = alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_;

  //force torque
  previous_balance_param_.foot_x_force_p_gain               = alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_;
  previous_balance_param_.foot_y_force_p_gain               = alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_;
  previous_balance_param_.foot_z_force_p_gain               = alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_;
  previous_balance_param_.foot_roll_torque_p_gain           = alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_;
  previous_balance_param_.foot_pitch_torque_p_gain          = alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_;

  previous_balance_param_.foot_x_force_d_gain               = alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_;
  previous_balance_param_.foot_y_force_d_gain               = alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_;
  previous_balance_param_.foot_z_force_d_gain               = alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_;
  previous_balance_param_.foot_roll_torque_d_gain           = alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_;
  previous_balance_param_.foot_pitch_torque_d_gain          = alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_;

  ////cut off freq
  //gyro
  previous_balance_param_.roll_gyro_cut_off_frequency  = alice_walking->balance_ctrl_.roll_gyro_lpf_.getCutOffFrequency();
  previous_balance_param_.pitch_gyro_cut_off_frequency = alice_walking->balance_ctrl_.pitch_gyro_lpf_.getCutOffFrequency();

  //orientation
  previous_balance_param_.roll_angle_cut_off_frequency  = alice_walking->balance_ctrl_.roll_angle_lpf_.getCutOffFrequency();
  previous_balance_param_.pitch_angle_cut_off_frequency = alice_walking->balance_ctrl_.pitch_angle_lpf_.getCutOffFrequency();

  //force torque
  previous_balance_param_.foot_x_force_cut_off_frequency     = alice_walking->balance_ctrl_.right_foot_force_x_lpf_.getCutOffFrequency();
  previous_balance_param_.foot_y_force_cut_off_frequency     = alice_walking->balance_ctrl_.right_foot_force_y_lpf_.getCutOffFrequency();
  previous_balance_param_.foot_z_force_cut_off_frequency     = alice_walking->balance_ctrl_.right_foot_force_z_lpf_.getCutOffFrequency();
  previous_balance_param_.foot_roll_torque_cut_off_frequency  = alice_walking->balance_ctrl_.right_foot_torque_roll_lpf_.getCutOffFrequency();
  previous_balance_param_.foot_pitch_torque_cut_off_frequency = alice_walking->balance_ctrl_.right_foot_torque_pitch_lpf_.getCutOffFrequency();

  balance_update_with_loop_ = true;

  std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_STARTED_MSG;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);

  return true;
}

void WalkingModule::setBalanceParam(alice_walking_module_msgs::BalanceParam& balance_param_msg)
{
  ALICEWalking *alice_walking = ALICEWalking::getInstance();

  alice_walking->balance_ctrl_.setCOBManualAdjustment(balance_param_msg.cob_x_offset_m, balance_param_msg.cob_y_offset_m, 0);

  //// set gain
  //gyro
  alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_ = balance_param_msg.foot_roll_gyro_p_gain;
  alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_ = balance_param_msg.foot_roll_gyro_d_gain;
  alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_ = balance_param_msg.foot_pitch_gyro_p_gain;
  alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_ = balance_param_msg.foot_pitch_gyro_d_gain;

  //orientation
  alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_  = balance_param_msg.foot_roll_angle_p_gain;
  alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_  = balance_param_msg.foot_roll_angle_d_gain;
  alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_ = balance_param_msg.foot_pitch_angle_p_gain;
  alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_ = balance_param_msg.foot_pitch_angle_d_gain;

  //force torque
  alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_      = balance_param_msg.foot_x_force_p_gain;
  alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_      = balance_param_msg.foot_y_force_p_gain;
  alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_      = balance_param_msg.foot_z_force_p_gain;
  alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_  = balance_param_msg.foot_roll_torque_p_gain;
  alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_ = balance_param_msg.foot_roll_torque_p_gain;
  alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_      = balance_param_msg.foot_x_force_d_gain;
  alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_      = balance_param_msg.foot_y_force_d_gain;
  alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_      = balance_param_msg.foot_z_force_d_gain;
  alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_  = balance_param_msg.foot_roll_torque_d_gain;
  alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_ = balance_param_msg.foot_roll_torque_d_gain;

  alice_walking->balance_ctrl_.left_foot_force_x_ctrl_.p_gain_      = balance_param_msg.foot_x_force_p_gain;
  alice_walking->balance_ctrl_.left_foot_force_y_ctrl_.p_gain_      = balance_param_msg.foot_y_force_p_gain;
  alice_walking->balance_ctrl_.left_foot_force_z_ctrl_.p_gain_      = balance_param_msg.foot_z_force_p_gain;
  alice_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.p_gain_  = balance_param_msg.foot_roll_torque_p_gain;
  alice_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.p_gain_ = balance_param_msg.foot_roll_torque_p_gain;
  alice_walking->balance_ctrl_.left_foot_force_x_ctrl_.d_gain_      = balance_param_msg.foot_x_force_d_gain;
  alice_walking->balance_ctrl_.left_foot_force_y_ctrl_.d_gain_      = balance_param_msg.foot_y_force_d_gain;
  alice_walking->balance_ctrl_.left_foot_force_z_ctrl_.d_gain_      = balance_param_msg.foot_z_force_d_gain;
  alice_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.d_gain_  = balance_param_msg.foot_roll_torque_d_gain;
  alice_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.d_gain_ = balance_param_msg.foot_roll_torque_d_gain;

  //// set cut off freq
  alice_walking->balance_ctrl_.roll_gyro_lpf_.setCutOffFrequency(balance_param_msg.roll_gyro_cut_off_frequency);
  alice_walking->balance_ctrl_.pitch_gyro_lpf_.setCutOffFrequency(balance_param_msg.pitch_gyro_cut_off_frequency);
  alice_walking->balance_ctrl_.roll_angle_lpf_.setCutOffFrequency(balance_param_msg.roll_angle_cut_off_frequency);
  alice_walking->balance_ctrl_.pitch_angle_lpf_.setCutOffFrequency(balance_param_msg.pitch_angle_cut_off_frequency);

  alice_walking->balance_ctrl_.right_foot_force_x_lpf_.setCutOffFrequency(balance_param_msg.foot_x_force_cut_off_frequency);
  alice_walking->balance_ctrl_.right_foot_force_y_lpf_.setCutOffFrequency(balance_param_msg.foot_y_force_cut_off_frequency);
  alice_walking->balance_ctrl_.right_foot_force_z_lpf_.setCutOffFrequency(balance_param_msg.foot_z_force_cut_off_frequency);
  alice_walking->balance_ctrl_.right_foot_torque_roll_lpf_.setCutOffFrequency(balance_param_msg.foot_roll_torque_cut_off_frequency);
  alice_walking->balance_ctrl_.right_foot_torque_pitch_lpf_.setCutOffFrequency(balance_param_msg.foot_pitch_torque_cut_off_frequency);

  alice_walking->balance_ctrl_.left_foot_force_x_lpf_.setCutOffFrequency(balance_param_msg.foot_x_force_cut_off_frequency);
  alice_walking->balance_ctrl_.left_foot_force_y_lpf_.setCutOffFrequency(balance_param_msg.foot_y_force_cut_off_frequency);
  alice_walking->balance_ctrl_.left_foot_force_z_lpf_.setCutOffFrequency(balance_param_msg.foot_z_force_cut_off_frequency);
  alice_walking->balance_ctrl_.left_foot_torque_roll_lpf_.setCutOffFrequency(balance_param_msg.foot_roll_torque_cut_off_frequency);
  alice_walking->balance_ctrl_.left_foot_torque_pitch_lpf_.setCutOffFrequency(balance_param_msg.foot_pitch_torque_cut_off_frequency);
}

void WalkingModule::updateBalanceParam()
{
  double current_update_gain = balance_update_tra_.getPosition(balance_update_sys_time_);

  current_balance_param_.cob_x_offset_m                  = current_update_gain*(desired_balance_param_.cob_x_offset_m                   - previous_balance_param_.cob_x_offset_m                ) + previous_balance_param_.cob_x_offset_m;
  current_balance_param_.cob_y_offset_m                  = current_update_gain*(desired_balance_param_.cob_y_offset_m                   - previous_balance_param_.cob_y_offset_m                ) + previous_balance_param_.cob_y_offset_m;

  current_balance_param_.foot_roll_gyro_p_gain                = current_update_gain*(desired_balance_param_.foot_roll_gyro_p_gain                - previous_balance_param_.foot_roll_gyro_p_gain              ) + previous_balance_param_.foot_roll_gyro_p_gain;
  current_balance_param_.foot_roll_gyro_d_gain                = current_update_gain*(desired_balance_param_.foot_roll_gyro_d_gain                - previous_balance_param_.foot_roll_gyro_d_gain              ) + previous_balance_param_.foot_roll_gyro_d_gain;
  current_balance_param_.foot_pitch_gyro_p_gain               = current_update_gain*(desired_balance_param_.foot_pitch_gyro_p_gain               - previous_balance_param_.foot_pitch_gyro_p_gain             ) + previous_balance_param_.foot_pitch_gyro_p_gain;
  current_balance_param_.foot_pitch_gyro_d_gain               = current_update_gain*(desired_balance_param_.foot_pitch_gyro_d_gain               - previous_balance_param_.foot_pitch_gyro_d_gain             ) + previous_balance_param_.foot_pitch_gyro_d_gain;
  current_balance_param_.foot_roll_angle_p_gain               = current_update_gain*(desired_balance_param_.foot_roll_angle_p_gain               - previous_balance_param_.foot_roll_angle_p_gain             ) + previous_balance_param_.foot_roll_angle_p_gain;
  current_balance_param_.foot_roll_angle_d_gain               = current_update_gain*(desired_balance_param_.foot_roll_angle_d_gain               - previous_balance_param_.foot_roll_angle_d_gain             ) + previous_balance_param_.foot_roll_angle_d_gain;
  current_balance_param_.foot_pitch_angle_p_gain              = current_update_gain*(desired_balance_param_.foot_pitch_angle_p_gain              - previous_balance_param_.foot_pitch_angle_p_gain            ) + previous_balance_param_.foot_pitch_angle_p_gain;
  current_balance_param_.foot_pitch_angle_d_gain              = current_update_gain*(desired_balance_param_.foot_pitch_angle_d_gain              - previous_balance_param_.foot_pitch_angle_d_gain            ) + previous_balance_param_.foot_pitch_angle_d_gain;
  current_balance_param_.foot_x_force_p_gain                  = current_update_gain*(desired_balance_param_.foot_x_force_p_gain                  - previous_balance_param_.foot_x_force_p_gain                ) + previous_balance_param_.foot_x_force_p_gain;
  current_balance_param_.foot_y_force_p_gain                  = current_update_gain*(desired_balance_param_.foot_y_force_p_gain                  - previous_balance_param_.foot_y_force_p_gain                ) + previous_balance_param_.foot_y_force_p_gain;
  current_balance_param_.foot_z_force_p_gain                  = current_update_gain*(desired_balance_param_.foot_z_force_p_gain                  - previous_balance_param_.foot_z_force_p_gain                ) + previous_balance_param_.foot_z_force_p_gain;
  current_balance_param_.foot_roll_torque_p_gain              = current_update_gain*(desired_balance_param_.foot_roll_torque_p_gain              - previous_balance_param_.foot_roll_torque_p_gain            ) + previous_balance_param_.foot_roll_torque_p_gain;
  current_balance_param_.foot_pitch_torque_p_gain             = current_update_gain*(desired_balance_param_.foot_pitch_torque_p_gain             - previous_balance_param_.foot_pitch_torque_p_gain           ) + previous_balance_param_.foot_pitch_torque_p_gain;
  current_balance_param_.foot_x_force_d_gain                  = current_update_gain*(desired_balance_param_.foot_x_force_d_gain                  - previous_balance_param_.foot_x_force_d_gain                ) + previous_balance_param_.foot_x_force_d_gain;
  current_balance_param_.foot_y_force_d_gain                  = current_update_gain*(desired_balance_param_.foot_y_force_d_gain                  - previous_balance_param_.foot_y_force_d_gain                ) + previous_balance_param_.foot_y_force_d_gain;
  current_balance_param_.foot_z_force_d_gain                  = current_update_gain*(desired_balance_param_.foot_z_force_d_gain                  - previous_balance_param_.foot_z_force_d_gain                ) + previous_balance_param_.foot_z_force_d_gain;
  current_balance_param_.foot_roll_torque_d_gain              = current_update_gain*(desired_balance_param_.foot_roll_torque_d_gain              - previous_balance_param_.foot_roll_torque_d_gain            ) + previous_balance_param_.foot_roll_torque_d_gain;
  current_balance_param_.foot_pitch_torque_d_gain             = current_update_gain*(desired_balance_param_.foot_pitch_torque_d_gain             - previous_balance_param_.foot_pitch_torque_d_gain           ) + previous_balance_param_.foot_pitch_torque_d_gain;

  current_balance_param_.roll_gyro_cut_off_frequency          = current_update_gain*(desired_balance_param_.roll_gyro_cut_off_frequency          - previous_balance_param_.roll_gyro_cut_off_frequency        ) + previous_balance_param_.roll_gyro_cut_off_frequency;
  current_balance_param_.pitch_gyro_cut_off_frequency         = current_update_gain*(desired_balance_param_.pitch_gyro_cut_off_frequency         - previous_balance_param_.pitch_gyro_cut_off_frequency       ) + previous_balance_param_.pitch_gyro_cut_off_frequency;
  current_balance_param_.roll_angle_cut_off_frequency         = current_update_gain*(desired_balance_param_.roll_angle_cut_off_frequency         - previous_balance_param_.roll_angle_cut_off_frequency       ) + previous_balance_param_.roll_angle_cut_off_frequency;
  current_balance_param_.pitch_angle_cut_off_frequency        = current_update_gain*(desired_balance_param_.pitch_angle_cut_off_frequency        - previous_balance_param_.pitch_angle_cut_off_frequency      ) + previous_balance_param_.pitch_angle_cut_off_frequency;
  current_balance_param_.foot_x_force_cut_off_frequency       = current_update_gain*(desired_balance_param_.foot_x_force_cut_off_frequency       - previous_balance_param_.foot_x_force_cut_off_frequency     ) + previous_balance_param_.foot_x_force_cut_off_frequency;
  current_balance_param_.foot_y_force_cut_off_frequency       = current_update_gain*(desired_balance_param_.foot_y_force_cut_off_frequency       - previous_balance_param_.foot_y_force_cut_off_frequency     ) + previous_balance_param_.foot_y_force_cut_off_frequency;
  current_balance_param_.foot_z_force_cut_off_frequency       = current_update_gain*(desired_balance_param_.foot_z_force_cut_off_frequency       - previous_balance_param_.foot_z_force_cut_off_frequency     ) + previous_balance_param_.foot_z_force_cut_off_frequency;
  current_balance_param_.foot_roll_torque_cut_off_frequency   = current_update_gain*(desired_balance_param_.foot_roll_torque_cut_off_frequency   - previous_balance_param_.foot_roll_torque_cut_off_frequency ) + previous_balance_param_.foot_roll_torque_cut_off_frequency;
  current_balance_param_.foot_pitch_torque_cut_off_frequency  = current_update_gain*(desired_balance_param_.foot_pitch_torque_cut_off_frequency  - previous_balance_param_.foot_pitch_torque_cut_off_frequency) + previous_balance_param_.foot_pitch_torque_cut_off_frequency;

  setBalanceParam(current_balance_param_);
}

bool WalkingModule::checkBalanceOnOff()
{
  return true;

  if(gazebo_)
    return true;

  ALICEWalking *alice_walking = ALICEWalking::getInstance();

  if ((fabs(alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_           ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_           ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_          ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_          ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_          ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_          ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_         ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_         ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_       ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_       ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_       ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_   ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_  ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_       ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_       ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_       ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_   ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_  ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_force_x_ctrl_.p_gain_        ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_force_y_ctrl_.p_gain_        ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_force_z_ctrl_.p_gain_        ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.p_gain_    ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.p_gain_   ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_force_x_ctrl_.d_gain_        ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_force_y_ctrl_.d_gain_        ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_force_z_ctrl_.d_gain_        ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.d_gain_    ) < 1e-7) &&
      (fabs(alice_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.d_gain_   ) < 1e-7))
  {
    return false;
  }
  else
    return true;
}


void WalkingModule::imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  ALICEWalking *alice_walking = ALICEWalking::getInstance();

  alice_walking->setCurrentIMUSensorOutput((msg->angular_velocity.y), (msg->angular_velocity.x),
      msg->orientation.x, msg->orientation.y, msg->orientation.z,
      msg->orientation.w);
}

void WalkingModule::ftDataOutputCallback(const alice_ft_sensor_msgs::ForceTorque::ConstPtr &msg)
{
  ALICEWalking *alice_walking = ALICEWalking::getInstance();

  alice_walking->setCurrentFTSensorOutput(msg->force_x_raw_r, msg->force_y_raw_r, msg->force_z_raw_r,
      msg->torque_x_raw_r, msg->torque_y_raw_r, msg->torque_z_raw_r,
      msg->force_x_raw_l, msg->force_y_raw_l, msg->force_z_raw_l,
      msg->torque_x_raw_l, msg->torque_y_raw_l, msg->torque_z_raw_l);
}


void WalkingModule::onModuleEnable()
{
  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void WalkingModule::onModuleDisable()
{
  previous_running_ = present_running = false;

  ALICEWalking *alice_walking = ALICEWalking::getInstance();
  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG;
  balance_update_with_loop_ = false;


  alice_walking->leg_angle_feed_back_[0].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[0].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[1].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[1].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[2].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[2].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[3].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[3].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[4].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[4].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[5].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[5].d_gain_ = 0;

  alice_walking->leg_angle_feed_back_[6].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[6].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[7].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[7].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[8].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[8].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[9].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[9].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[10].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[10].d_gain_ = 0;
  alice_walking->leg_angle_feed_back_[11].p_gain_ = 0;
  alice_walking->leg_angle_feed_back_[11].d_gain_ = 0;

  alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_           = 0;
  alice_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_           = 0;
  alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_          = 0;
  alice_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_          = 0;
  alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_          = 0;
  alice_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_          = 0;
  alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_         = 0;
  alice_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_         = 0;
  alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_       = 0;
  alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_       = 0;
  alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_       = 0;
  alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_   = 0;
  alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_  = 0;
  alice_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_       = 0;
  alice_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_       = 0;
  alice_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_       = 0;
  alice_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_   = 0;
  alice_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_  = 0;
  alice_walking->balance_ctrl_.left_foot_force_x_ctrl_.p_gain_        = 0;
  alice_walking->balance_ctrl_.left_foot_force_y_ctrl_.p_gain_        = 0;
  alice_walking->balance_ctrl_.left_foot_force_z_ctrl_.p_gain_        = 0;
  alice_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.p_gain_    = 0;
  alice_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.p_gain_   = 0;
  alice_walking->balance_ctrl_.left_foot_force_x_ctrl_.d_gain_        = 0;
  alice_walking->balance_ctrl_.left_foot_force_y_ctrl_.d_gain_        = 0;
  alice_walking->balance_ctrl_.left_foot_force_z_ctrl_.d_gain_        = 0;
  alice_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.d_gain_    = 0;
  alice_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.d_gain_   = 0;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void WalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{

  if(enable_ == false)
    return;

  ALICEWalking *alice_walking = ALICEWalking::getInstance();

  if(alice_id_ == "3")
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
  
  if(balance_update_with_loop_ == true)
  {
    balance_update_sys_time_ += control_cycle_msec_ * 0.001;
    if(balance_update_sys_time_ >= balance_update_duration_ )
    {
      balance_update_sys_time_ = balance_update_duration_;
      balance_update_with_loop_ = false;
      setBalanceParam(desired_balance_param_);
      std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISHED_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      publishDoneMsg("walking_balance");
    }
    else
    {
      updateBalanceParam();
    }
  }

  if(joint_feedback_update_with_loop_ == true)
  {
    joint_feedback_update_sys_time_ += control_cycle_msec_ * 0.001;
    if(joint_feedback_update_sys_time_ >= joint_feedback_update_duration_ )
    {
      joint_feedback_update_sys_time_ = joint_feedback_update_duration_;
      joint_feedback_update_with_loop_ = false;
      setJointFeedBackGain(desired_joint_feedback_gain_);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, WalkingStatusMSG::JOINT_FEEDBACK_GAIN_UPDATE_FINISHED_MSG);
      publishDoneMsg("walking_joint_feedback");
    }
    else
    {
      updateJointFeedBackGain();
    }
  }

  for(std::map<std::string, robotis_framework::DynamixelState*>::iterator result_it = result_.begin();
      result_it != result_.end();
      result_it++)
  {
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxls_it = dxls.find(result_it->first);
    if(dxls_it != dxls.end())
      alice_walking->curr_angle_rad_[joint_name_to_index_[result_it->first]] = dxls_it->second->dxl_state_->present_position_;
  }

  process_mutex_.lock();
  
  alice_walking->process();

  process_mutex_.unlock();

  result_["r_hip_pitch"  ]->goal_position_ = alice_walking->out_angle_rad_[0];
  result_["r_hip_roll"   ]->goal_position_ = alice_walking->out_angle_rad_[1];
  result_["r_hip_yaw"    ]->goal_position_ = alice_walking->out_angle_rad_[2];
  result_["r_knee_pitch" ]->goal_position_ = alice_walking->out_angle_rad_[3];
  result_["r_ankle_pitch"]->goal_position_ = alice_walking->out_angle_rad_[4];
  result_["r_ankle_roll" ]->goal_position_ = alice_walking->out_angle_rad_[5];

  result_["l_hip_pitch"  ]->goal_position_ = alice_walking->out_angle_rad_[6];
  result_["l_hip_roll"   ]->goal_position_ = alice_walking->out_angle_rad_[7];
  result_["l_hip_yaw"    ]->goal_position_ = alice_walking->out_angle_rad_[8];
  result_["l_knee_pitch" ]->goal_position_ = alice_walking->out_angle_rad_[9];
  result_["l_ankle_pitch"]->goal_position_ = alice_walking->out_angle_rad_[10];
  result_["l_ankle_roll" ]->goal_position_ = alice_walking->out_angle_rad_[11];

  //GUI Publish
  reference_zmp_msg_.x = alice_walking->reference_zmp_.coeff(0);
  reference_zmp_msg_.y = alice_walking->reference_zmp_.coeff(1);
  reference_zmp_msg_.z = alice_walking->reference_zmp_.coeff(2);
  reference_zmp_pub_.publish(reference_zmp_msg_);

  reference_body_msg_.linear.x = alice_walking->reference_body_.x;
  reference_body_msg_.linear.y = alice_walking->reference_body_.y;
  reference_body_msg_.linear.z = alice_walking->reference_body_.z;
  reference_body_msg_.angular.x = alice_walking->reference_body_.roll;
  reference_body_msg_.angular.y = alice_walking->reference_body_.pitch;
  reference_body_msg_.angular.z = alice_walking->reference_body_.yaw;
  reference_body_pub_.publish(reference_body_msg_);

  if (alice_walking->publish_flag_reference_body_localization == true)
  {
    reference_body_delta_msg_.linear.x = alice_walking->delta_reference_body_localization_.x;
    reference_body_delta_msg_.linear.y = alice_walking->delta_reference_body_localization_.y;
    reference_body_delta_msg_.linear.z = alice_walking->delta_reference_body_localization_.z;
    reference_body_delta_msg_.angular.x = alice_walking->delta_reference_body_localization_.roll;
    reference_body_delta_msg_.angular.y = alice_walking->delta_reference_body_localization_.pitch;
    reference_body_delta_msg_.angular.z = alice_walking->delta_reference_body_localization_.yaw;
    reference_body_delta_pub_.publish(reference_body_delta_msg_);
    alice_walking->set_publish_flag_reference_body_localization_false();
  }

  // FT calc ZMP
  FTBasedZmpCalculate(alice_walking->mat_g_to_rfoot_, alice_walking->mat_g_to_lfoot_, alice_walking->mat_g_right_force_, alice_walking->mat_g_left_force_ , alice_walking->mat_g_right_torque_, alice_walking->mat_g_left_torque_);
  ft_calc_zmp_pub_.publish(ft_calc_zmp_msg);

  // IMU sensor position
  angle_sensor_msg_.x = alice_walking->current_imu_roll_rad_;
  angle_sensor_msg_.y = alice_walking->current_imu_pitch_rad_;
  angle_sensor_msg_.z = 0;
  angle_sensor_pub_.publish(angle_sensor_msg_);

  angle_acc_sensor_msg_.x = alice_walking->current_gyro_roll_rad_per_sec_;
  angle_acc_sensor_msg_.y = alice_walking->current_gyro_pitch_rad_per_sec_;
  angle_acc_sensor_msg_.z = 0; 
  angle_acc_sensor_pub_.publish(angle_acc_sensor_msg_);
  
  //foot publish
  foot_right_msg_.x = alice_walking->mat_g_to_rfoot_(0,3);
  foot_right_msg_.y = alice_walking->mat_g_to_rfoot_(1,3);
  foot_right_msg_.z = alice_walking->mat_g_to_rfoot_(2,3);
  foot_right_pub_.publish(foot_right_msg_);

  foot_left_msg_.x = alice_walking->mat_g_to_lfoot_(0,3);
  foot_left_msg_.y = alice_walking->mat_g_to_lfoot_(1,3);
  foot_left_msg_.z = alice_walking->mat_g_to_lfoot_(2,3);
  foot_left_pub_.publish(foot_left_msg_);

  present_running = isRunning();
  if(previous_running_ != present_running)
  {
    if(present_running == true)
    {
      //double secs =ros::Time::now().toSec();
      //ROS_INFO("Walking Start : %f",secs);
      std::string status_msg = WalkingStatusMSG::WALKING_START_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      //double secs =ros::Time::now().toSec();
      //ROS_INFO("test end : %f",secs);
      std::string status_msg = WalkingStatusMSG::WALKING_FINISH_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      publishDoneMsg("walking_completed");
    }
  }
  previous_running_ = present_running;
}

//yitaek zmp output
void WalkingModule::FTBasedZmpCalculate(Eigen::Matrix4d g_right_foot, Eigen::Matrix4d g_left_foot, Eigen::MatrixXd g_right_force, Eigen::MatrixXd g_left_force , Eigen::MatrixXd g_right_torque, Eigen::MatrixXd g_left_torque)
{
  geometry_msgs::Point right_zmp_,left_zmp_;

  right_zmp_.x = (-(g_right_torque(1,0))+ (leg_to_end_*g_right_force(0,0)))/(g_right_force(2,0));
  right_zmp_.y = ((g_right_torque(0,0))+ (leg_to_end_*g_right_force(1,0)))/(g_right_force(2,0));
  right_zmp_.z = 0;

  left_zmp_.x = (-(g_left_torque(1,0))+ (leg_to_end_*g_left_force(0,0)))/(g_left_force(2,0));
  left_zmp_.y = ((g_left_torque(0,0))+ (leg_to_end_*g_left_force(1,0)))/(g_left_force(2,0));
  left_zmp_.z = 0; 

  ft_calc_zmp_msg.x = (right_zmp_.x*g_right_force(2,0) + left_zmp_.x*g_left_force(2,0))/(g_right_force(2,0) + g_left_force(2,0));
  ft_calc_zmp_msg.y = (right_zmp_.y*g_right_force(2,0) + left_zmp_.y*g_left_force(2,0))/(g_right_force(2,0) + g_left_force(2,0));

}

void WalkingModule::stop()
{
  return;
}

