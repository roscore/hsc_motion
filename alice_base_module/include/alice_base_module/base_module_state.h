
#ifndef ALICE_BASE_MODULE_BASE_MODULE_STATE_H_
#define ALICE_BASE_MODULE_BASE_MODULE_STATE_H_

#include <eigen3/Eigen/Eigen>
#include "robotis_math/robotis_math.h"
#include "alice_kinematics_dynamics/kinematics_dynamics.h"

namespace alice
{

class BaseModuleState
{
public:
  BaseModuleState();
  ~BaseModuleState();

  bool is_moving_;

  int cnt_; // counter number

  double mov_time_; // movement time
  double smp_time_; // sampling time

  int all_time_steps_; // all time steps of movement time

  Eigen::MatrixXd calc_joint_tra_; // calculated joint trajectory

  Eigen::MatrixXd joint_ini_pose_;
  Eigen::MatrixXd joint_pose_;

};

}

#endif /* ALICE_BASE_MODULE_BASE_MODULE_STATE_H_ */