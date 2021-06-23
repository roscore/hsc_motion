

#include "alice_walking_test_module/alice_walking_test.h"

using namespace alice;


ALICEWalking::ALICEWalking()
{
  readKinematicsYamlData();
  alice_kd_ = 0;

  mat_pelvis_to_rhip_ = robotis_framework::getTransformationXYZRPY(0, -pelvis_to_hip_, 0, 0, 0, 0);  
  mat_rhip_to_pelvis_ = robotis_framework::getTransformationXYZRPY(0,  pelvis_to_hip_, 0, 0, 0, 0);

  mat_pelvis_to_lhip_ = robotis_framework::getTransformationXYZRPY(0,  pelvis_to_hip_, 0, 0, 0, 0);
  mat_lhip_to_pelvis_ = robotis_framework::getTransformationXYZRPY(0, -pelvis_to_hip_, 0, 0, 0, 0);

  balance_error_ = heroehs::BalanceControlError::NoError;

  mat_imu_frame_ref_ = robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5*M_PI);
  mat_imu_frame_ref_inv_ = mat_imu_frame_ref_.transpose();

  total_robot_mass_ = total_mass_;
  right_dsp_fz_N_ = -0.5* total_robot_mass_ * 9.8;
  left_dsp_fz_N_  = -0.5* total_robot_mass_ * 9.8;
  right_ssp_fz_N_ = -total_robot_mass_ * 9.8;
  left_ssp_fz_N_  = -total_robot_mass_ * 9.8;

  current_imu_roll_rad_ = current_imu_pitch_rad_ = 0;
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;

  current_right_fx_N_  =  current_right_fy_N_  = current_right_fz_N_  =0;
  current_right_tx_Nm_ =  current_right_ty_Nm_ = current_right_tz_Nm_ =0;
  current_left_fx_N_   = current_left_fy_N_    = current_left_fz_N_   =0;
  current_left_tx_Nm_  = current_left_ty_Nm_   = current_left_tz_Nm_  =0;

  balance_index_ = 0;

  mat_g_to_acc_.resize(4, 1);
  mat_g_to_acc_.fill(0);

  for(int i = 0; i<12; i++)
  {
    curr_angle_rad_[i] = 0;
  }


}

ALICEWalking::~ALICEWalking()
{

}

void ALICEWalking::readKinematicsYamlData()
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
  pelvis_to_hip_ = kinematics_doc["l_leg_hip_p"]["relative_position"][1].as<double>();
  total_mass_= kinematics_doc["total_mass"].as<double>();
  online_walking_pelvis_h_= kinematics_doc["online_walking_pelvis_h"].as<double>();
  lipm_height_m_=kinematics_doc["lipm_height_m"].as<double>();
  default_foot_y_offset_=kinematics_doc["default_y_offset_foot"].as<double>();

  std::string walking_path = ros::package::getPath("heroehs_walking_pattern_generator_test")+"/config/data_"+alice_id+".yaml";
  YAML::Node walking_doc;
  try
  {
    walking_doc = YAML::LoadFile(walking_path.c_str());

  }catch(const std::exception& e)
  {
    ROS_ERROR("[Walking - alice walking] Fail to load walking yaml file!");
    return;
  }
  preview_time_sec_ = walking_doc["preview_time"].as<double>();

}



void ALICEWalking::initialize(double control_cycle_sec)
{
  alice_kd_ = new KinematicsDynamics();

  robotis_framework::Pose3D r_foot, l_foot, pelvis;
  r_foot.x = 0.0;    r_foot.y = -default_foot_y_offset_;  r_foot.z = -online_walking_pelvis_h_;
  r_foot.roll = 0.0; r_foot.pitch = 0.0; r_foot.yaw = 0;

  l_foot.x = 0.0;    l_foot.y = default_foot_y_offset_;   l_foot.z = -online_walking_pelvis_h_;
  l_foot.roll = 0.0; l_foot.pitch = 0.0; l_foot.yaw = 0;

  pelvis.x = 0.0;    pelvis.y = 0.0;     pelvis.z = 0.0;
  pelvis.roll = 0.0; pelvis.pitch = 0.0; pelvis.yaw = 0;

  pose_calculator_.setInitialPose(r_foot, l_foot, pelvis); 
  
  pose_calculator_.initialize(lipm_height_m_, preview_time_sec_, control_cycle_sec);


  // initialize balance
  balance_ctrl_.initialize(control_cycle_sec);
  balance_ctrl_.setGyroBalanceEnable(true);
  balance_ctrl_.setOrientationBalanceEnable(true);
  balance_ctrl_.setForceTorqueBalanceEnable(true);

  mat_right_force_.resize(4,1);  mat_left_force_.resize(4,1);
  mat_right_torque_.resize(4,1); mat_left_torque_.resize(4,1);

  mat_right_force_.fill(0);  mat_left_force_.fill(0);
  mat_right_torque_.fill(0); mat_left_torque_.fill(0);


  //ground to force
  mat_g_right_force_.resize(4,1); mat_g_left_force_.resize(4,1);
  mat_g_right_force_.fill(0); mat_g_left_force_.fill(0);

  mat_g_right_torque_.resize(4,1); mat_g_left_torque_.resize(4,1);
  mat_g_right_torque_.fill(0); mat_g_left_torque_.fill(0);

  for(int feed_forward_idx = 0; feed_forward_idx < 12; feed_forward_idx++)
  {
    leg_angle_feed_back_[feed_forward_idx].p_gain_ = 0;
    leg_angle_feed_back_[feed_forward_idx].d_gain_ = 0;
  }

  mat_g_to_acc_.resize(4, 1);
  mat_g_to_acc_.fill(0);


}

void ALICEWalking::start()
{
  pose_calculator_.start();
}

void ALICEWalking::addStepData(robotis_framework::StepData& step_data)
{
  pose_calculator_.addStepData(step_data);
}

void ALICEWalking::eraseLastStepData()
{
  pose_calculator_.eraseLastStepData();
}

int ALICEWalking::getNumofRemainingUnreservedStepData()
{
  return pose_calculator_.getNumofRemainingAllStepData();
  //return pose_calculator_.getNumofRemainingUnreservedStepData();
}

void ALICEWalking::getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition)
{
  pose_calculator_.getReferenceStepDatafotAddition(ref_step_data_for_addition);
}


void ALICEWalking::set_publish_flag_reference_body_localization_false()
{
  pose_calculator_.publish_flag_reference_body_localization=false;
}

void ALICEWalking::process()
{

  pose_calculator_.calcDesiredPose();

  reference_zmp_=pose_calculator_.reference_zmp_;
  reference_body_=pose_calculator_.reference_body_;
  delta_reference_body_localization_ = pose_calculator_.delta_reference_body_localization;
  publish_flag_reference_body_localization = pose_calculator_.publish_flag_reference_body_localization;

  mat_g_to_rfoot_  = robotis_framework::getTransformationXYZRPY(pose_calculator_.present_right_foot_pose_.x, pose_calculator_.present_right_foot_pose_.y, pose_calculator_.present_right_foot_pose_.z,
      pose_calculator_.present_right_foot_pose_.roll, pose_calculator_.present_right_foot_pose_.pitch, pose_calculator_.present_right_foot_pose_.yaw);
  mat_g_to_lfoot_  = robotis_framework::getTransformationXYZRPY(pose_calculator_.present_left_foot_pose_.x, pose_calculator_.present_left_foot_pose_.y, pose_calculator_.present_left_foot_pose_.z,
      pose_calculator_.present_left_foot_pose_.roll, pose_calculator_.present_left_foot_pose_.pitch, pose_calculator_.present_left_foot_pose_.yaw);
  mat_g_to_pelvis_ = robotis_framework::getTransformationXYZRPY(pose_calculator_.present_body_pose_.x, pose_calculator_.present_body_pose_.y, pose_calculator_.present_body_pose_.z,
      pose_calculator_.present_body_pose_.roll, pose_calculator_.present_body_pose_.pitch, pose_calculator_.present_body_pose_.yaw);

  pose_g_to_pelvis_ = pose_calculator_.present_body_pose_;
  mat_pelvis_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_pelvis_);

  mat_robot_to_pelvis_ = robotis_framework::getRotation4d(pose_calculator_.present_body_pose_.roll, pose_calculator_.present_body_pose_.pitch, 0);
  mat_pelvis_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_pelvis_);

  mat_g_to_robot_ = mat_g_to_pelvis_* mat_pelvis_to_robot_;
  mat_robot_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_robot_);

  mat_robot_to_rfoot_ = mat_robot_to_g_*mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_robot_to_g_*mat_g_to_lfoot_;

  
  //balance

  imu_data_mutex_lock_.lock();
  balance_ctrl_.setCurrentGyroSensorOutput(current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_);
  balance_ctrl_.setCurrentOrientationSensorOutput(current_imu_roll_rad_, current_imu_pitch_rad_);
  imu_data_mutex_lock_.unlock();
  ft_data_mutex_lock_.lock();

  balance_ctrl_.setCurrentFootForceTorqueSensorOutput(mat_right_force_.coeff(0,0),  mat_right_force_.coeff(1,0),  mat_right_force_.coeff(2,0),
      mat_right_torque_.coeff(0,0), mat_right_torque_.coeff(1,0), mat_right_torque_.coeff(2,0),
      mat_left_force_.coeff(0,0),   mat_left_force_.coeff(1,0),   mat_left_force_.coeff(2,0),
      mat_left_torque_.coeff(0,0),  mat_left_torque_.coeff(1,0),  mat_left_torque_.coeff(2,0));
  ft_data_mutex_lock_.unlock();

  balance_index_ = pose_calculator_.current_balancing_index_;
  double r_target_fx_N = 0;
  double l_target_fx_N = 0;
  double r_target_fy_N = 0;
  double l_target_fy_N = 0;
  double r_target_fz_N = right_dsp_fz_N_;
  double l_target_fz_N = left_dsp_fz_N_;
  double target_fz_N  = 0;
  mat_g_to_acc_.coeffRef(0,0) = pose_calculator_.x_lipm_.coeff(2,0);
  mat_g_to_acc_.coeffRef(1,0) = pose_calculator_.y_lipm_.coeff(2,0);
  mat_robot_to_acc_ = (mat_robot_to_pelvis_* mat_pelvis_to_g_) * mat_g_to_acc_;

  switch(balance_index_)
  {
  case 0:
    //fprintf(stderr, "DSP : START\n");
    //std::cout << "DSP START : " << pose_calculator_.walking_time_ << std::endl;

    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    r_target_fz_N = right_dsp_fz_N_;
    l_target_fz_N = left_dsp_fz_N_;
    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
    break;
  case 1:
    //fprintf(stderr, "DSP : R--O->L\n");
    //std::cout << "DSP : R--O->L : " << pose_calculator_.walking_time_ << std::endl;

    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    r_target_fz_N = right_dsp_fz_N_;
    l_target_fz_N = left_dsp_fz_N_;
    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
    break;
  case 2:
    //fprintf(stderr, "SSP : L_BALANCING1\n");
    //std::cout << "SSP : L_BALANCING1 : " << pose_calculator_.walking_time_ << std::endl;
    
    r_target_fx_N = 0;
    r_target_fy_N = 0;
    r_target_fz_N = 0;

    l_target_fx_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    l_target_fy_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    l_target_fz_N = left_ssp_fz_N_;
    target_fz_N = left_ssp_fz_N_;
    break;
  case 3:
    //std::cout << "SSP : L_BALANCING2 : " << pose_calculator_.walking_time_ << std::endl;
    //fprintf(stderr, "SSP : L_BALANCING2\n");
    
    r_target_fx_N = 0;
    r_target_fy_N = 0;
    r_target_fz_N = 0;

    l_target_fx_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    l_target_fy_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    l_target_fz_N = left_ssp_fz_N_;
    target_fz_N = left_ssp_fz_N_;
    break;
  case 4:
    //fprintf(stderr, "DSP : R--O<-L\n");
    //std::cout << "DSP : R--O<-L : " << pose_calculator_.walking_time_ << std::endl;
    
    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    r_target_fz_N = right_dsp_fz_N_;
    l_target_fz_N = left_dsp_fz_N_;
    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
    break;
  case 5:
    //fprintf(stderr, "DSP : R<-O--L\n");
    //std::cout << "DSP : R<-O--L : " << pose_calculator_.walking_time_ << std::endl;
    
    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    r_target_fz_N = right_dsp_fz_N_;
    l_target_fz_N = left_dsp_fz_N_;
    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
    break;
  case 6:
    //fprintf(stderr, "SSP : R_BALANCING1\n");
    //std::cout << "SSP : R_BALANCING1 : " << pose_calculator_.walking_time_ << std::endl;
    
    r_target_fx_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    r_target_fy_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    r_target_fz_N = right_ssp_fz_N_;

    l_target_fx_N = 0;
    l_target_fy_N = 0;
    l_target_fz_N = 0;
    target_fz_N = -right_ssp_fz_N_;
    break;
  case 7:
    //fprintf(stderr, "SSP : R_BALANCING2\n");
    //std::cout << "SSP : R_BALANCING2 : " << pose_calculator_.walking_time_ << std::endl;
    
    r_target_fx_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    r_target_fy_N = -1.0*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    r_target_fz_N = right_ssp_fz_N_;

    l_target_fx_N = 0;
    l_target_fy_N = 0;
    l_target_fz_N = 0;
    target_fz_N =  -right_ssp_fz_N_;
    break;
  case 8:
    //fprintf(stderr, "DSP : R->O--L");
    //std::cout << "DSP : R->O--L : " << pose_calculator_.walking_time_ << std::endl;
    
    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    r_target_fz_N = right_dsp_fz_N_;
    l_target_fz_N = left_dsp_fz_N_;
    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
    break;
  case 9:
    //fprintf(stderr, "DSP : END");
    //std::cout << "DSP : END : " << pose_calculator_.walking_time_ << std::endl;
    
    r_target_fx_N = l_target_fx_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(0,0);
    r_target_fy_N = l_target_fy_N = -0.5*total_robot_mass_*mat_robot_to_acc_.coeff(1,0);
    r_target_fz_N = right_dsp_fz_N_;
    l_target_fz_N = left_dsp_fz_N_;
    target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
    break;
  default:
    break;
  }


  l_target_fz_N = pose_calculator_.switching_ratio_*left_dsp_fz_N_ + left_dsp_fz_N_;
  r_target_fz_N = right_ssp_fz_N_ - l_target_fz_N;
  

  balance_ctrl_.setDesiredCOBGyro(0,0);
  balance_ctrl_.setDesiredCOBOrientation(pose_g_to_pelvis_.roll, pose_g_to_pelvis_.pitch);
  balance_ctrl_.setDesiredFootForceTorque(r_target_fx_N*1.0, r_target_fy_N*1.0, r_target_fz_N, 0, 0, 0,
      l_target_fx_N*1.0, l_target_fy_N*1.0, l_target_fz_N, 0, 0, 0);
  balance_ctrl_.setDesiredPose(mat_robot_to_pelvis_, mat_robot_to_rfoot_, mat_robot_to_lfoot_);

  balance_ctrl_.process(&balance_error_, &mat_robot_to_pelvis_modified_, &mat_robot_to_rf_modified_, &mat_robot_to_lf_modified_);
  mat_pelvis_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_pelvis_modified_);

  rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_pelvis_ * mat_pelvis_to_robot_modified_) * mat_robot_to_rf_modified_);
  lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_pelvis_ * mat_pelvis_to_robot_modified_) * mat_robot_to_lf_modified_);

  alice_kd_->calcInverseKinematicsForRightLeg(r_leg_out_angle_rad_, rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z,
      rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
  alice_kd_->calcInverseKinematicsForLeftLeg(l_leg_out_angle_rad_, lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z,
      lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);

  for(int i = 0; i < 6; i++)
  {
    out_angle_rad_[i+0] = r_leg_out_angle_rad_[i];
    out_angle_rad_[i+6] = l_leg_out_angle_rad_[i];

  }


  alice_kd_->alice_link_data_[15]->joint_angle_ = out_angle_rad_[6];
  alice_kd_->alice_link_data_[13]->joint_angle_ = out_angle_rad_[7];
  alice_kd_->alice_link_data_[11]->joint_angle_ = out_angle_rad_[8];

  alice_kd_->alice_link_data_[17]->joint_angle_ = out_angle_rad_[9];
  alice_kd_->alice_link_data_[19]->joint_angle_ = out_angle_rad_[10];
  alice_kd_->alice_link_data_[21]->joint_angle_ = out_angle_rad_[11];

  alice_kd_->alice_link_data_[16]->joint_angle_ = out_angle_rad_[0];
  alice_kd_->alice_link_data_[14]->joint_angle_ = out_angle_rad_[1];
  alice_kd_->alice_link_data_[12]->joint_angle_ = out_angle_rad_[2];

  alice_kd_->alice_link_data_[18]->joint_angle_ = out_angle_rad_[3];
  alice_kd_->alice_link_data_[20]->joint_angle_ = out_angle_rad_[4];
  alice_kd_->alice_link_data_[22]->joint_angle_ = out_angle_rad_[5];


  for(int angle_idx = 0; angle_idx < 6; angle_idx++)
  {
    leg_angle_feed_back_[angle_idx+0].desired_ = out_angle_rad_[angle_idx];
    leg_angle_feed_back_[angle_idx+6].desired_ = out_angle_rad_[angle_idx+6];
    out_angle_rad_[angle_idx+0] = out_angle_rad_[angle_idx+0] + leg_angle_feed_back_[angle_idx+0].getFeedBack(curr_angle_rad_[angle_idx]);
    out_angle_rad_[angle_idx+6] = out_angle_rad_[angle_idx+6] + leg_angle_feed_back_[angle_idx+6].getFeedBack(curr_angle_rad_[angle_idx+6]);

  }
}

bool ALICEWalking::isRunning()
{
  return pose_calculator_.isRunning();
}


void ALICEWalking::setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w)
{
  imu_data_mutex_lock_.lock();

  current_gyro_roll_rad_per_sec_  = gyro_x;
  current_gyro_pitch_rad_per_sec_ = gyro_y;

  quat_current_imu_ = Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);

  mat_current_imu_ = (mat_imu_frame_ref_ * quat_current_imu_.toRotationMatrix()) * mat_imu_frame_ref_inv_;

  current_imu_roll_rad_  = atan2( mat_current_imu_.coeff(2,1), mat_current_imu_.coeff(2,2));
  current_imu_pitch_rad_ = atan2(-mat_current_imu_.coeff(2,0), sqrt(robotis_framework::powDI(mat_current_imu_.coeff(2,1), 2) + robotis_framework::powDI(mat_current_imu_.coeff(2,2), 2)));


  imu_data_mutex_lock_.unlock();
}

void ALICEWalking::setCurrentFTSensorOutput(double rfx, double rfy, double rfz, double rtx, double rty, double rtz,
    double lfx, double lfy, double lfz, double ltx, double lty, double ltz)
{
  ft_data_mutex_lock_.lock();
  current_right_fx_N_ = rfx;  current_right_fy_N_ = rfy;  current_right_fz_N_ = rfz;
  current_right_tx_Nm_= rtx; current_right_ty_Nm_= rty; current_right_tz_Nm_= rtz;

  current_left_fx_N_ = lfx;  current_left_fy_N_ = lfy;  current_left_fz_N_ = lfz;
  current_left_tx_Nm_ = ltx; current_left_ty_Nm_ = lty; current_left_tz_Nm_ = ltz;

  mat_right_force_(0,0) = current_right_fx_N_;
  mat_right_force_(1,0) = current_right_fy_N_;
  mat_right_force_(2,0) = current_right_fz_N_;
  mat_left_force_(0,0)  = current_left_fx_N_;
  mat_left_force_(1,0)  = current_left_fy_N_;
  mat_left_force_(2,0)  = current_left_fz_N_;

  mat_right_torque_(0,0) = current_right_tx_Nm_;
  mat_right_torque_(1,0) = current_right_ty_Nm_;
  mat_right_torque_(2,0) = current_right_tz_Nm_;
  mat_left_torque_(0,0) = current_left_tx_Nm_;
  mat_left_torque_(1,0) = current_left_ty_Nm_;
  mat_left_torque_(2,0) = current_left_tz_Nm_;

  //  mat_right_force_  = mat_robot_to_rfoot_*mat_right_force_;
  // mat_right_torque_ = mat_robot_to_rfoot_*mat_right_torque_;

  //  mat_right_force_ = mat_robot_right_foot_*mat_right_force_;
  //  mat_right_torque_= mat_robot_right_foot_*mat_right_torque_;

  mat_g_right_force_= mat_right_force_;
  mat_g_right_torque_ = mat_right_torque_;

  // mat_left_force_  = mat_robot_to_lfoot_*mat_left_force_;
  // mat_left_torque_ = mat_robot_to_lfoot_*mat_left_torque_;
  //  mat_left_force_  = mat_robot_left_foot_*mat_left_force_;
  //  mat_left_torque_ = mat_robot_left_foot_*mat_left_torque_;

  mat_g_left_force_= mat_left_force_;
  mat_g_left_torque_ = mat_left_torque_;

  ft_data_mutex_lock_.unlock();
}



