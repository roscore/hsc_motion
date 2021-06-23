
#ifndef ALICE_WALKING_MODULE_ALICE_WALKING_H_
#define ALICE_WALKING_MODULE_ALICE_WALKING_H_

#include "robotis_math/robotis_math.h"
#include "alice_kinematics_dynamics/kinematics_dynamics.h"
#include "heroehs_walking_pattern_generator_test/heroehs_endpoint_calculator.h"
#include "heroehs_pd_balance_controller/heroehs_pd_balance_controller.h"
#include "robotis_framework_common/singleton.h"

#include <string>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

namespace alice
{

  class ALICEWalking : public robotis_framework::Singleton<ALICEWalking>
  {
  public:
    ALICEWalking();
    virtual ~ALICEWalking();

    void initialize(double control_cycle_sec);
    void start();

    void process();
    bool isRunning();

    void readKinematicsYamlData();
    double pelvis_to_hip_, total_mass_;
    double online_walking_pelvis_h_;
    double lipm_height_m_;
    double default_foot_y_offset_;
    double preview_time_sec_;

    void addStepData(robotis_framework::StepData &step_data);
    void eraseLastStepData();
    int getNumofRemainingUnreservedStepData();
    void getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition);

    Eigen::Matrix4d mat_pelvis_to_rhip_, mat_rhip_to_pelvis_;
    Eigen::Matrix4d mat_pelvis_to_lhip_, mat_lhip_to_pelvis_;
    Eigen::Matrix4d mat_g_to_pelvis_, mat_pelvis_to_g_;
    Eigen::Matrix4d mat_g_to_rfoot_, mat_g_to_lfoot_;
    Eigen::Matrix4d mat_robot_to_pelvis_, mat_pelvis_to_robot_;
    Eigen::Matrix4d mat_robot_to_rfoot_, mat_robot_to_lfoot_;
    Eigen::Matrix4d mat_g_to_robot_, mat_robot_to_g_;

    robotis_framework::Pose3D pose_g_to_pelvis_;

    Eigen::MatrixXd mat_robot_to_pelvis_modified_, mat_robot_to_rf_modified_, mat_robot_to_lf_modified_;
    Eigen::MatrixXd mat_pelvis_to_robot_modified_;

    robotis_framework::Pose3D rhip_to_rfoot_pose_, lhip_to_lfoot_pose_;

    Eigen::MatrixXd mat_g_to_acc_, mat_robot_to_acc_;

    double r_leg_out_angle_rad_[6];
    double l_leg_out_angle_rad_[6];
    double out_angle_rad_[12];
    double curr_angle_rad_[12];

    heroehs::PDController leg_angle_feed_back_[12];

    // FOR PUBLISH
    Eigen::Vector3d reference_zmp_;
    robotis_framework::Pose3D delta_reference_body_localization_;
    robotis_framework::Pose3D reference_body_;
    bool publish_flag_reference_body_localization;
    void set_publish_flag_reference_body_localization_false();

    // balance control
    int balance_index_;
    int balance_error_;
    heroehs::BalanceControlUsingPDController balance_ctrl_;

    void setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w);
    void setCurrentFTSensorOutput(double rfx, double rfy, double rfz, double rtx, double rty, double rtz,
                                  double lfx, double lfy, double lfz, double ltx, double lty, double ltz);

    // sensor value
    //imu
    Eigen::Quaterniond quat_current_imu_;
    Eigen::Matrix3d mat_current_imu_;
    Eigen::Matrix3d mat_imu_frame_ref_, mat_imu_frame_ref_inv_;
    double current_imu_roll_rad_, current_imu_pitch_rad_;
    double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;

    //torque
    double current_right_fx_N_, current_right_fy_N_, current_right_fz_N_;
    double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
    double current_left_fx_N_, current_left_fy_N_, current_left_fz_N_;
    double current_left_tx_Nm_, current_left_ty_Nm_, current_left_tz_Nm_;
    Eigen::MatrixXd mat_right_force_, mat_left_force_;
    Eigen::MatrixXd mat_g_right_force_, mat_g_left_force_;
    Eigen::MatrixXd mat_right_torque_, mat_left_torque_;
    Eigen::MatrixXd mat_g_right_torque_, mat_g_left_torque_;

  private:
    heroehs::EndPointCalculatorPreview pose_calculator_;
    KinematicsDynamics *alice_kd_;

    double total_robot_mass_;
    double right_dsp_fz_N_, left_dsp_fz_N_;
    double right_ssp_fz_N_, left_ssp_fz_N_;

    boost::mutex imu_data_mutex_lock_;
    boost::mutex ft_data_mutex_lock_;

  };

} // namespace alice

#endif /* ALICE_WALKING_MODULE_ALICE_WALKING_H_ */
