/*
 * alice_kinematics_dynamics.cpp
 *
 *  Created on: Jun 3, 2018
 *      Author: jaysong
 */

#include "alice_kinematics_dynamics/kinematics_dynamics.h"

using namespace alice;

KinematicsDynamics::KinematicsDynamics()
{

  readKinematicsYamlData();

  for (int id=0; id<=ALL_JOINT_ID; id++)
    alice_link_data_[id] = new LinkData();

  for (int link_id=0;link_id<ALL_JOINT_ID;link_id++)
  {
    YAML::Node joint_node = kinematics_doc_[kine_link_name_[link_id]];

    double jlmax = joint_node["joint_limit_max"].as<double>();
    double jlmin = joint_node["joint_limit_min"].as<double>();
    if(jlmax>-50 && jlmax<50) jlmax=jlmax*M_PI;
    if(jlmin>-50 && jlmin<50) jlmin=jlmin*M_PI;

    std::vector<double> rp = joint_node["relative_position"].as<std::vector<double> >();
    std::vector<double> ja = joint_node["joint_axis"].as<std::vector<double> >();
    std::vector<double> cm = joint_node["center_of_mass"].as<std::vector<double> >();
    std::vector<double> ia = joint_node["inertia"].as<std::vector<double> >();


    alice_link_data_[link_id]->name_               =  kine_link_name_[link_id];
    alice_link_data_[link_id]->parent_             =  joint_node["parent"].as<double>();
    alice_link_data_[link_id]->sibling_            =  joint_node["sibling"].as<double>();
    alice_link_data_[link_id]->child_              =  joint_node["child"].as<double>();
    alice_link_data_[link_id]->mass_               =  joint_node["mass"].as<double>();
    alice_link_data_[link_id]->relative_position_  =  robotis_framework::getTransitionXYZ( rp[0] , rp[1] , rp[2] );
    alice_link_data_[link_id]->joint_axis_         =  robotis_framework::getTransitionXYZ( ja[0] , ja[1] , ja[2] );
    alice_link_data_[link_id]->center_of_mass_     =  robotis_framework::getTransitionXYZ( cm[0] , cm[1] , cm[2] );
    alice_link_data_[link_id]->joint_limit_max_    =  jlmax;
    alice_link_data_[link_id]->joint_limit_min_    =  jlmin;
    alice_link_data_[link_id]->inertia_            =  robotis_framework::getInertiaXYZ( ia[0] , ia[1] , ia[2] , ia[3] , ia[4] , ia[5] );

  }
  thigh_length_m_= kinematics_doc_["thigh_length_m"].as<double>();
  calf_length_m_ = kinematics_doc_["calf_length_m"].as<double>();
  ankle_length_m_ = kinematics_doc_["ankle_length_m"].as<double>();
  leg_side_offset_m_ = kinematics_doc_["leg_side_offset_m"].as<double>();

  KinematicsGraig();
}

void KinematicsDynamics::readKinematicsYamlData()
{
  ros::NodeHandle nh;
  int alice_id_int  = nh.param<int>("alice_userid",0);

  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  std::string alice_id = alice_id_stream.str();

  std::string kinematics_path = ros::package::getPath("alice_kinematics_dynamics")+"/data/kin_dyn_"+alice_id+".yaml";

  try
  {
    kinematics_doc_ = YAML::LoadFile(kinematics_path.c_str());

  }catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load kinematics yaml file!");
    return;
  }
  //ROS_INFO("printed_kinematics yaml data!");
  //ROS_INFO("%s",kinematics_path.c_str());
   
  std::string temp_kine_link_name[ALL_JOINT_ID]={
      "base",
      //motor 1-22
      "l_arm_sh_p1","r_arm_sh_p1", "l_arm_sh_r", "r_arm_sh_r", "l_arm_el_y", "r_arm_el_y",
      "head_p", "head_y", "torso_y", "torso_p",
      "l_leg_hip_y", "r_leg_hip_y", "l_leg_hip_r", "r_leg_hip_r", "l_leg_hip_p","r_leg_hip_p",
      "l_leg_kn_p", "r_leg_kn_p", "l_leg_an_p", "r_leg_an_p", "l_leg_an_r", "r_leg_an_r",
      //end point 23-26
      "l_arm_end", "r_arm_end", "l_leg_end", "r_leg_end",
      //other 27-34
      "cam", "pelvis",
      "passive_x", "passive_y", "passive_z", "passive_yaw", "passive_pitch", "passive_roll"
  };

  for(int t=0;t<ALL_JOINT_ID;t++)
    kine_link_name_[t]=temp_kine_link_name[t];

}


double KinematicsDynamics::calcTotalMass(int joint_id)
{
  double mass;

  if (joint_id == -1)
    mass = 0.0;
  else
    mass = alice_link_data_[joint_id]->mass_ + calcTotalMass(alice_link_data_[ joint_id ]->sibling_) + calcTotalMass(alice_link_data_[joint_id]->child_);

  return mass;
}

Eigen::MatrixXd KinematicsDynamics::calcMassCenter(int joint_id)
{
  Eigen::MatrixXd mc(3,1);

  if (joint_id == -1)
    mc = Eigen::MatrixXd::Zero(3,1);
  else
  {
    mc = alice_link_data_[ joint_id ]->mass_ * ( alice_link_data_[ joint_id ]->orientation_ * alice_link_data_[ joint_id ]->center_of_mass_ + alice_link_data_[ joint_id ]->position_ );
    mc = mc + calcMassCenter( alice_link_data_[ joint_id ]->sibling_ ) + calcMassCenter( alice_link_data_[ joint_id ]->child_ );
  }

  return mc;
}

void KinematicsDynamics::calcJointsCenterOfMass(int joint_id)
{
  if(joint_id != -1)
  {
    LinkData *temp_data = alice_link_data_[ joint_id ];
    temp_data->joint_center_of_mass_
    = ( temp_data->orientation_ * temp_data->center_of_mass_ + temp_data->position_ );

    calcJointsCenterOfMass(temp_data->sibling_);
    calcJointsCenterOfMass(temp_data->child_);
  }
  else
    return;
}


Eigen::MatrixXd KinematicsDynamics::calcCenterOfMass(Eigen::MatrixXd mc)
{
  double mass ;
  Eigen::MatrixXd COM(3,1);

  mass = calcTotalMass(0);
  COM = mc/mass;

  return COM;
}

void KinematicsDynamics::calcForwardKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == 0)
  {
    alice_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3,1);
    alice_link_data_[0]->orientation_ =
        robotis_framework::calcRodrigues( robotis_framework::calcHatto( alice_link_data_[0]->joint_axis_ ), alice_link_data_[ 0 ]->joint_angle_ );
  }

  if ( joint_id != 0 )
  {
    int parent = alice_link_data_[joint_id]->parent_;

    alice_link_data_[joint_id]->position_ =
        alice_link_data_[parent]->orientation_ * alice_link_data_[joint_id]->relative_position_ + alice_link_data_[parent]->position_;
    alice_link_data_[ joint_id ]->orientation_ =
        alice_link_data_[ parent ]->orientation_ *
        robotis_framework::calcRodrigues(robotis_framework::calcHatto(alice_link_data_[joint_id]->joint_axis_), alice_link_data_[joint_id]->joint_angle_);

    //alice_link_data_[joint_id]->transformation_.block<3,1>(0,3) = alice_link_data_[joint_id]->position_;
    //alice_link_data_[joint_id]->transformation_.block<3,3>(0,0) = alice_link_data_[joint_id]->orientation_;
  }

  calcForwardKinematics(alice_link_data_[joint_id]->sibling_);
  calcForwardKinematics(alice_link_data_[joint_id]->child_);
}

bool KinematicsDynamics::calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Matrix4d trans_ad, trans_da, trans_cd, trans_dc;
  Eigen::Matrix3d rot_ac;
  Eigen::Vector3d vec;

  bool invertible;
  double rac, arc_cos, arc_tan, alpha;
  double thigh_length = thigh_length_m_;
  double calf_length = calf_length_m_;
  double ankle_length = ankle_length_m_;

  trans_ad = robotis_framework::getTransformationXYZRPY(x, y, z, roll, pitch, yaw);

  vec.coeffRef(0) = trans_ad.coeff(0, 3) + trans_ad.coeff(0, 2) * ankle_length;
  vec.coeffRef(1) = trans_ad.coeff(1, 3) + trans_ad.coeff(1, 2) * ankle_length;
  vec.coeffRef(2) = trans_ad.coeff(2, 3) + trans_ad.coeff(2, 2) * ankle_length;

  // Get Knee
  rac = vec.norm();
  arc_cos = acos(
      (rac * rac - thigh_length * thigh_length - calf_length * calf_length) / (2.0 * thigh_length * calf_length));
  if (std::isnan(arc_cos) == 1)
    return false;
  *(out + 3) = arc_cos;

  // Get Ankle Roll
  trans_ad.computeInverseWithCheck(trans_da, invertible);
  if (invertible == false)
    return false;

  vec.coeffRef(0) = trans_da.coeff(0, 3);
  vec.coeffRef(1) = trans_da.coeff(1, 3);
  vec.coeffRef(2) = trans_da.coeff(2, 3) - ankle_length;

  arc_tan = atan2(vec(1), vec(2));
  if(arc_tan > M_PI_2)
    arc_tan = arc_tan - M_PI;
  else if(arc_tan < -M_PI_2)
    arc_tan = arc_tan + M_PI;

  *(out+5) = arc_tan;

  //Get Ankle Pitch
  alpha = asin( thigh_length*sin(M_PI - *(out+3)) / rac);
  *(out+4) = -atan2(vec(0), copysign(sqrt(vec(1)*vec(1) + vec(2)*vec(2)),vec(2))) - alpha;

  // Get Hip Pitch
  rot_ac = ( robotis_framework::convertRPYToRotation(roll, pitch, yaw)*robotis_framework::getRotationX(-(*(out+5))))
                  * robotis_framework::getRotationY(-(*(out+3) + *(out+4)));

  arc_tan = atan2(rot_ac.coeff(0, 2), rot_ac.coeff(2, 2));
  *(out) = arc_tan;

  // Get Hip Roll
  arc_tan = atan2(-rot_ac.coeff(1, 2), rot_ac.coeff(0, 2) * sin(*(out)) + rot_ac.coeff(2, 2) * cos(*(out)));
  *(out + 1) = arc_tan;

  // Get Hip Yaw
  arc_tan = atan2(rot_ac.coeff(1, 0), rot_ac.coeff(1, 1));
  *(out+2) = arc_tan;


  return true;
}

bool KinematicsDynamics::calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  if(calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true) {

    *(out + 0) = out[0] * (alice_link_data_[ID_R_LEG_START + 2*0]->joint_axis_.coeff(1,0));
    *(out + 1) = out[1] * (alice_link_data_[ID_R_LEG_START + 2*1]->joint_axis_.coeff(0,0));
    *(out + 2) = out[2] * (alice_link_data_[ID_R_LEG_START + 2*2]->joint_axis_.coeff(2,0));
    *(out + 3) = out[3] * (alice_link_data_[ID_R_LEG_START + 2*3]->joint_axis_.coeff(1,0));
    *(out + 4) = out[4] * (alice_link_data_[ID_R_LEG_START + 2*4]->joint_axis_.coeff(1,0));
    *(out + 5) = out[5] * (alice_link_data_[ID_R_LEG_START + 2*5]->joint_axis_.coeff(0,0));
    return true;
  }
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  if(calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true) {

    out[0] = out[0] * (alice_link_data_[ID_L_LEG_START + 2*0]->joint_axis_.coeff(1,0));
    out[1] = out[1] * (alice_link_data_[ID_L_LEG_START + 2*1]->joint_axis_.coeff(0,0));
    out[2] = out[2] * (alice_link_data_[ID_L_LEG_START + 2*2]->joint_axis_.coeff(2,0));
    out[3] = out[3] * (alice_link_data_[ID_L_LEG_START + 2*3]->joint_axis_.coeff(1,0));
    out[4] = out[4] * (alice_link_data_[ID_L_LEG_START + 2*4]->joint_axis_.coeff(1,0));
    out[5] = out[5] * (alice_link_data_[ID_L_LEG_START + 2*5]->joint_axis_.coeff(0,0));
    return true;
  }
  else
    return false;
}

bool KinematicsDynamics::KinematicsGraig()
{
  joint_radian.resize(7,1);
  joint_radian.fill(0);

  // kinematics variables //
  P_inverse_.fill(0);
  P_.fill(0);

  // DH convention variables
  dh_alpha[0] = 0;
  dh_alpha[1] = M_PI/2;
  dh_alpha[2] = -M_PI/2;
  dh_alpha[3] = -M_PI/2;
  dh_alpha[4] = 0;
  dh_alpha[5] = M_PI/2;
  dh_alpha[6] = 0;

  dh_link[0] = 0;
  dh_link[1] = 0;
  dh_link[2] = 0;
  dh_link[3] = 0;
  dh_link[4] = 0.240;
  dh_link[5] = 0;
  dh_link[6] = 0.127;

  total_length_ = 0.607;
  sensor_length_ = 0.127;

  dh_link_d[0] = 0;
  dh_link_d[1] = 0;
  dh_link_d[2] = 0;
  dh_link_d[3] = 0.240;
  dh_link_d[4] = 0;
  dh_link_d[5] = 0;
  dh_link_d[6] = 0;

  real_theta[0] = 0;
  real_theta[1] = 0;
  real_theta[2] = 0;
  real_theta[3] = 0;
  real_theta[4] = 0;
  real_theta[5] = 0;
  real_theta[6] = 0;

  for(int i=0; i<8;i++)
  {
    H[i].resize(4,4);
    H[i].fill(0);
  }
  center_to_sensor_transform_right.resize(4,4);
  center_to_sensor_transform_right.fill(0);
  center_to_sensor_transform_left.resize(4,4);
  center_to_sensor_transform_left.fill(0);
  center_to_foot_transform_left_leg.resize(4,4);
  center_to_foot_transform_left_leg.fill(0);
  center_to_foot_transform_right_leg.resize(4,4);
  center_to_foot_transform_right_leg.fill(0);
  H_ground_to_center.resize(4,4);
  H_ground_to_center.fill(0);

  H[7] << 0 , 0, -1, 0,
      0 , 1,  0, 0,
      1 , 0,  0, 0,
      0 , 0,  0, 1;

  H_ground_to_center << 1 , 0, 0, 0,
      0 , 1, 0, 0,
      0 , 0, 1, total_length_,
      0 , 0, 0, 1;


}
void KinematicsDynamics::FowardKinematics(double joint[7], std::string left_right)
{
  double sum_theta[7] = {0,0,0,0,0,0,0};
  double offset_theta[7] = {0, 0, (M_PI)/2, -(M_PI)/2, (M_PI)/2, 0, 0};

  for(int i=1; i<7; i++)
  {
    sum_theta[i] = joint[i]+ offset_theta[i];
  }
  for(int i=1; i<7; i++)
  {
    //Homogenous Transformation
    H[i](0,0) = floor(100000.*(cos(sum_theta[i])+0.000005))/100000.;
    H[i](0,1) = floor(100000.*(-cos(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
    H[i](0,2) = floor(100000.*(sin(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
    H[i](0,3) = floor(100000.*(dh_link[i]*cos(sum_theta[i])+0.000005))/100000.;

    H[i](1,0) = floor(100000.*(sin(sum_theta[i])+0.000005))/100000.;
    H[i](1,1) = floor(100000.*(cos(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
    H[i](1,2) = floor(100000.*(-sin(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
    H[i](1,3) = floor(100000.*(dh_link[i]*sin(sum_theta[i])+0.000005))/100000.;

    H[i](2,0) =0;
    H[i](2,1) = floor(100000.*(sin(dh_alpha[i])+0.000005))/100000.;
    H[i](2,2) = floor(100000.*(cos(dh_alpha[i])+0.000005))/100000.;
    H[i](2,3) = -dh_link_d[i];

    H[i](3,0) =0;
    H[i](3,1) =0;
    H[i](3,2) =0;
    H[i](3,3) =1;

    H[i](0,0) = cos(sum_theta[i]);
    H[i](0,1) = -cos(dh_alpha[i])*sin(sum_theta[i]);
    H[i](0,2) = sin(dh_alpha[i])*sin(sum_theta[i]);
    H[i](0,3) = dh_link[i]*cos(sum_theta[i]);

    H[i](1,0) = sin(sum_theta[i]);
    H[i](1,1) = cos(dh_alpha[i])*cos(sum_theta[i]);
    H[i](1,2) = -sin(dh_alpha[i])*cos(sum_theta[i]);
    H[i](1,3) = dh_link[i]*sin(sum_theta[i]);

    H[i](2,0) =0;
    H[i](2,1) = sin(dh_alpha[i]);
    H[i](2,2) = cos(dh_alpha[i]);
    H[i](2,3) = -dh_link_d[i];

    H[i](3,0) =0;
    H[i](3,1) =0;
    H[i](3,2) =0;
    H[i](3,3) =1;
  }

  H[0](0,0) = 0;
  H[0](0,1) = -1;
  H[0](0,2) = 0;
  H[0](0,3) = 0;

  H[0](1,0) = 0;
  H[0](1,1) = 0;
  H[0](1,2) = 1;
  H[0](1,3) = -0.09;

  H[0](2,0) = -1;
  H[0](2,1) = 0;
  H[0](2,2) = 0;
  H[0](2,3) = 0;

  H[0](3,0) = 0;
  H[0](3,1) = 0;
  H[0](3,2) = 0;
  H[0](3,3) = 1;
  //// foot frame 을 pelvis frame 과 일치 시킨다.
  if(!left_right.compare("left")) // left
  {
    H[0](1,3) = 0.09;
    center_to_foot_transform_left_leg = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6]*H[7];
  }
  else // right
  {
    H[0](1,3) = -0.09;
    center_to_foot_transform_right_leg = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6]*H[7];
  }
}



