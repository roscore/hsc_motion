/*
 * pidcontroller.h
 *
 *  Created on: Aug 7, 2020
 *      Author: heroehs
 */

#ifndef ALICE_HEROEHS_ALICE_MOTION_ALICE_TORSO_MODULE_INCLUDE_ALICE_TORSO_MODULE_PIDCONTROLLER_MODULE_H_
#define ALICE_HEROEHS_ALICE_MOTION_ALICE_TORSO_MODULE_INCLUDE_ALICE_TORSO_MODULE_PIDCONTROLLER_MODULE_H_

namespace alice
{

	class PIDController
	{
	public:
		PIDController(double control_time = 0.008);
		~PIDController();

		double PID_process(double desired, double present);

		void PID_set_gains(double Kp, double Ki, double Kd);
		void PID_reset_integral();

		double kp, ki, kd;

	private:

		double control_time_;
		double current_error_;
		double previous_error_;
		double integrator_;
	};

}


#endif /* ALICE_HEROEHS_ALICE_MOTION_ALICE_TORSO_MODULE_INCLUDE_ALICE_TORSO_MODULE_PIDCONTROLLER_MODULE_H_ */
