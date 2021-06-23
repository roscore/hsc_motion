/*
 * fifth_order_trajectory_generate.cpp
 *
 *  Created on: 2017. 10. 23.
 *      Author: RobotEmperor
 */


#include "alice_torso_module/fifth_order_trajectory_generator.h"


using namespace alice;
using namespace robotis_framework;

FifthOrderTrajectory::FifthOrderTrajectory(double control_time)
{

    control_time_ = control_time;

    current_time = 0;
    is_moving_traj = false;

    initial_time = 0;
    initial_pose = 0;
    initial_velocity = 0;
    initial_acc = 0;

    current_time = 0;
    current_pose = 0;
    current_velocity = 0;
    current_acc = 0;

    final_time = 0;
    final_pose = 0;
    final_velocity = 0;
    final_acc = 0;

    position_coeff_.resize(6, 1);
    velocity_coeff_.resize(6, 1);
    acceleration_coeff_.resize(6, 1);
    time_variables_.resize(1, 6);

    position_coeff_.fill(0);
    velocity_coeff_.fill(0);
    acceleration_coeff_.fill(0);
    time_variables_.fill(0);

}


FifthOrderTrajectory::FifthOrderTrajectory()
{
    control_time_ = 0;

    current_time = 0;
    is_moving_traj = false;

    initial_time = 0;
    initial_pose = 0;
    initial_velocity = 0;
    initial_acc = 0;

    current_time = 0;
    current_pose = 0;
    current_velocity = 0;
    current_acc = 0;

    final_time = 0;
    final_pose = 0;
    final_velocity = 0;
    final_acc = 0;

    position_coeff_.resize(6, 1);
    velocity_coeff_.resize(6, 1);
    acceleration_coeff_.resize(6, 1);
    time_variables_.resize(1, 6);

    position_coeff_.fill(0);
    velocity_coeff_.fill(0);
    acceleration_coeff_.fill(0);
    time_variables_.fill(0);

}

FifthOrderTrajectory::~FifthOrderTrajectory()
{

}

bool FifthOrderTrajectory::detect_change_final_value(double pose_, double velocity_, double time_)
{
    if (pose_ != final_pose || velocity_ != final_velocity || time_ != final_time)
    {
        final_pose = pose_;
        final_velocity = velocity_;
        final_time = time_;
        current_time = 0;
        return true;
    }
    else
        return false;

}

double FifthOrderTrajectory::fifth_order_traj_gen(double initial_value_, double final_value_,
    double initial_velocity_, double final_velocity_,
    double initial_acc_, double final_acc_,
    double initial_time_, double final_time_)
{
    if (current_time == 0)
    {
        final_pose = final_value_;
        final_velocity = final_velocity_;
        final_acc = final_acc_;
        final_time = final_time_;

        initial_value_ = current_pose;
        initial_velocity_ = current_velocity;
        initial_acc_ = current_acc;

        Eigen::MatrixXd time_mat;
        Eigen::MatrixXd conditions_mat;

        time_mat.resize(6, 6);
        time_mat.fill(0);
        time_mat << powDI(initial_time_, 5), powDI(initial_time_, 4), powDI(initial_time_, 3), powDI(initial_time_, 2), initial_time_, 1.0,
            5.0 * powDI(initial_time_, 4), 4.0 * powDI(initial_time_, 3), 3.0 * powDI(initial_time_, 2), 2.0 * initial_time_, 1.0, 0.0,
            20.0 * powDI(initial_time_, 3), 12.0 * powDI(initial_time_, 2), 6.0 * initial_time_, 2.0, 0.0, 0.0,
            powDI(final_time_, 5), powDI(final_time_, 4), powDI(final_time_, 3), powDI(final_time_, 2), final_time_, 1.0,
            5.0 * powDI(final_time_, 4), 4.0 * powDI(final_time_, 3), 3.0 * powDI(final_time_, 2), 2.0 * final_time_, 1.0, 0.0,
            20.0 * powDI(final_time_, 3), 12.0 * powDI(final_time_, 2), 6.0 * final_time_, 2.0, 0.0, 0.0;

        conditions_mat.resize(6, 1);
        conditions_mat.fill(0);
        conditions_mat << initial_value_, initial_velocity_, initial_acc_, final_value_, final_velocity_, final_acc_;

        position_coeff_ = time_mat.inverse() * conditions_mat;


        velocity_coeff_ << 0.0,
            5.0 * position_coeff_.coeff(0, 0),
            4.0 * position_coeff_.coeff(1, 0),
            3.0 * position_coeff_.coeff(2, 0),
            2.0 * position_coeff_.coeff(3, 0),
            1.0 * position_coeff_.coeff(4, 0);
        acceleration_coeff_ << 0.0,
            0.0,
            20.0 * position_coeff_.coeff(0, 0),
            12.0 * position_coeff_.coeff(1, 0),
            6.0 * position_coeff_.coeff(2, 0),
            2.0 * position_coeff_.coeff(3, 0);


        is_moving_traj = true;
    }

    if (current_time <= final_time_)
    {
        current_time = current_time + 0.008;
        time_variables_ << powDI(current_time, 5), powDI(current_time, 4), powDI(current_time, 3), powDI(current_time, 2), current_time, 1.0;
        current_pose = (time_variables_ * position_coeff_).coeff(0, 0);
        current_velocity = (time_variables_ * velocity_coeff_).coeff(0, 0);
        current_acc = (time_variables_ * acceleration_coeff_).coeff(0, 0);

        is_moving_traj = true;

        return current_pose;
    }
    else
    {
        is_moving_traj = false;
        return current_pose;
    }
}

double FifthOrderTrajectory::fifth_order_traj_gen_one_value(Eigen::MatrixXd joint_)
{
    double result_one_joint_;

    if (detect_change_final_value(joint_(0, 1), joint_(0, 3), joint_(0, 7)))
    {
        current_time = 0;
        ROS_INFO("One Value Change!");
    }
    result_one_joint_ = fifth_order_traj_gen(current_pose, joint_(0, 1), current_velocity, joint_(0, 3), joint_(0, 4), joint_(0, 5), joint_(0, 6), joint_(0, 7));

    return result_one_joint_;
}
