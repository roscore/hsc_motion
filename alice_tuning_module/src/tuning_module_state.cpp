
#include "alice_tuning_module/tuning_module_state.h"

namespace alice
{
TuningModuleState::TuningModuleState(int via_num)
{
  is_moving_ = false;
  is_generating_ = false;

  cnt_ = 0;

  mov_time_ = 1.0;
  smp_time_ = 0.008;
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;

  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);

  joint_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

}

TuningModuleState::~TuningModuleState()
{
}

}
