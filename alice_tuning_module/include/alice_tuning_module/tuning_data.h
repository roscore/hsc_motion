#ifndef ALICE_TUNING_MODULE_TUNING_DATA_H_
#define ALICE_TUNING_MODULE_TUNING_DATA_H_

#include <string>

namespace alice
{

template<typename T>
class JointElement
{
public:
  void getValue(T &value)
  {
    if(has_value_)
      value = joint_value_;
  }
  void setValue(const T &value)
  {
    joint_value_ = value;
    has_value_ = true;
  }
  void clear()
  {
    has_value_ = false;
  }

private:
  T joint_value_;
  bool has_value_;
};

class TuningData
{
public:
  TuningData();
  ~TuningData();
  void clearData();

  JointElement<std::string> joint_name_;

  JointElement<double> position_;

};

class JointOffsetData
{
 public:
  double joint_offset_rad_;
  double goal_position_;

  JointOffsetData()
  {
    joint_offset_rad_ = 0;
    goal_position_ = 0;

  }

  JointOffsetData(double joint_offset_rad, double goal_position)
  {
    this->joint_offset_rad_ = joint_offset_rad;
    this->goal_position_ = goal_position;

  }

  JointOffsetData(double joint_offset_rad, double goal_position, int p_gain, int i_gain, int d_gain)
  {
    this->joint_offset_rad_ = joint_offset_rad;
    this->goal_position_ = goal_position;

  }

  ~JointOffsetData()
  {
  }
};
}

#endif /* ALICE_TUNING_MODULE_TUNING_DATA_H_ */
