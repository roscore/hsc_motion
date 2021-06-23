/*
 * alice_torso_module.cpp
 *
 *  Created on: Aug 7, 2020
 *      Author: heroehs
 */

#include <stdio.h>
#include <cmath>
#include "alice_torso_module/alice_torso_module.h"

using namespace alice;

TorsoModule::TorsoModule()
: control_cycle_msec_(8)
{
	running_ 		= false;
	enable_       	= false;
	gazebo_check  	= false;
	module_name_  	= "torso_module";
	control_mode_ 	= robotis_framework::PositionControl;

	readIDData();

	// Dynamixel initialize ////
	if(alice_id_ == "3")
	{
		result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 13
		result_["head_pitch"]       = new robotis_framework::DynamixelState();  // joint 14
		result_["waist_yaw"]		= new robotis_framework::DynamixelState();  // joint 9
	}
	else if(alice_id_ == "2")
	{
		result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 13
		result_["head_pitch"]       = new robotis_framework::DynamixelState();  // joint 14
	}
	else if(alice_id_ == "1")
	{
		result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 13
		result_["head_pitch"]       = new robotis_framework::DynamixelState();  // joint 14
		result_["waist_yaw"]		= new robotis_framework::DynamixelState();  // joint 9
	}
	else
		ROS_WARN("Unknown Alice ID!!!");

	//init
	new_count_ = 1;
	is_moving_state = false;

	mode_ = 7;
	pre_mode_ = 7;
	
	search_phase = 0;

	/* motion */

	// search_phase : 1
	search_motion[0][0] = 0.523599;		// Degree -> 30
	search_motion[0][1] = 0.785398;		// Degree -> 45

	// search_phase : 2
	search_motion[1][0] = 0.523599;		// Degree -> 30
	search_motion[1][1] = -0.785398;	// Degree -> 45

	// search_phase : 3
	search_motion[2][0] = 0.523599;		// Degree -> 20
	search_motion[2][1] = 0.0;

	// search_phase : 4
	search_motion[3][0] = 0.872665;		// Degree -> 20
	search_motion[3][1] = 0.0;

	// search_phase : 5
	search_motion[4][0] = 0.872665;		// Degree -> 20
	search_motion[4][1] = 0.0;

	// search_phase : 6
	search_motion[5][0] = 0.523599;		// Degree -> 20
	search_motion[5][1] = 0.0;

	motion_bias = 0.0349066;			// Degree -> 2

	max_limit_[0]=45; // 20
	//max_limit_[1]=40; // 40
	//max_limit_[2]=30;

	error_[0] = 0;
	error_[1] = 0;
	error_[2] = 0;

	min_p_gain[0] = 0.15; 
	min_p_gain[1] = 0.15;
	min_p_gain[2] = 0.15;

	max_p_gain[0] = 0.3;
	max_p_gain[1] = 0.3;
	max_p_gain[2] = 0.2;

	// head pitch
	max_error_[0] = 0.872665;
	min_error_[0] = 0;

	// head yaw
	max_error_[1] = 0.698132;
	min_error_[1] = 0;

	// waist yaw
	max_error_[2] = 1.5708;
	min_error_[2] = 0;

	mapped_error_[0] = min_p_gain[0];
	mapped_error_[1] = min_p_gain[1];
	mapped_error_[2] = min_p_gain[2];

	joint_name_to_id_.clear();
	joint_id_to_name_.clear();

	joint_name_to_curr_pose_.clear();

	// Manual
	joint_id_to_rad_.clear();

	// Tracking
	joint_name_to_goal_pose_.clear();

	// Searching
	joint_name_to_ini_pose_state_.clear();
	joint_name_to_ini_pose_goal_.clear();
	//joint_name_to_check_.clear();
	motion_phase_init = false;

	max_boxsize_ = 0;
	min_boxsize_ = 0;

	x_resolution = 1280;
	y_resolution = 720;

	x_origin = x_resolution/2;
	y_origin = y_resolution/2;
	
	roi_x_offset=0;
	roi_y_offset=0;
	roi_height=0;
	roi_width=0;

	y, pre_y, pre_x = 0;
	ts = 0.001;
}

TorsoModule::~TorsoModule()
{
	queue_thread_.join();
}

void TorsoModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&TorsoModule::queueThread, this));

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name]		= dxl_info->id_;
		joint_id_to_name_[dxl_info->id_]	= joint_name;
		joint_id_to_rad_[dxl_info->id_] 	= 0;

		dxl_pidcontroller[joint_name] = new PIDController(); //for manual
		dxl_pidcontroller[joint_name]->PID_set_gains(0.12,0,0);

	}

	motion_trajectory[joint_id_to_name_[7]]	= new alice::FifthOrderTrajectory();
  	motion_trajectory[joint_id_to_name_[8]]	= new alice::FifthOrderTrajectory();

	mov_time_state = 2.0;

	dxl_pidcontroller[joint_id_to_name_[7]]->PID_set_gains(0.25,0,0.00); //0.0015
	dxl_pidcontroller[joint_id_to_name_[8]]->PID_set_gains(0.15,0,0.00); //202007 기준 0.1,0,0.00 +  dxl_init.yaml 에서 P gain -> 2배로 증가시킴

	if(alice_id_ == "3" || alice_id_ == "1")
	{
		dxl_pidcontroller[joint_id_to_name_[9]]->PID_set_gains(0.1,0,0);
		motion_trajectory[joint_id_to_name_[9]]	= new alice::FifthOrderTrajectory();
	}

	ROS_INFO("< -------  Initialize Module : Torso Module !!  ------->");
}


void TorsoModule::torsomodeCallback(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("=================================\n");

	pre_mode_ = mode_;

	if(msg->data == 0)
	{
		mode_ = 0;
		if(pre_mode_ != 0)
		{
			new_count_ = 1;
			is_moving_state = false;
			ROS_INFO("		Auto Mode\n");
		}
	}
	else if(msg->data == 1)
	{
		mode_ = 1;
		if(pre_mode_ != 1)
		{
			new_count_ = 1;
			is_moving_state = false;
			ROS_INFO("		Manual Mode\n");
		}
	}
	else if(msg->data == 2)
	{
		mode_ = 2;
		if((pre_mode_ != 2) || (search_phase == 0))
		{
			new_count_ = 1;
			search_phase = 1;
			is_moving_state = false;
			motion_phase_init = true;
			ROS_INFO("		Searching Mode\n");
		}
	}
	else if(msg->data == 3)
	{
		mode_ = 3;
		if(pre_mode_ != 3)
		{
			new_count_ = 1;
			is_moving_state = false;
			ROS_INFO("		Tracking Mode\n");
		}
	}
	else if(msg->data == 4)
	{
		mode_ = 4;
		new_count_ = 1;
		is_moving_state = false;
		ROS_INFO("		Scan Mode\n");
	}
	else if(msg->data == 5)
	{
		mode_ = 5;
		if((pre_mode_ != 5) || (search_phase == 0))
		{
			new_count_ = 1;
			search_phase = 5;
			is_moving_state = false;
			motion_phase_init = true;
			ROS_INFO("		Ball_check Mode\n");
		}
	}
	else if(msg->data == 6)
	{
		mode_ = 6;
		if(pre_mode_ != 6)
		{
			new_count_ = 1;
			is_moving_state = false;
			ROS_INFO("		Torso Module :: STOP !!\n");
		}
	}
}

void TorsoModule::manualMovingCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	if(mode_ != 1)
	{
		return;
	}

	joint_id_to_rad_[7] = DEG2RAD_(msg->data[0]);
	joint_id_to_rad_[8] = DEG2RAD_(msg->data[1]);

	//ROS_INFO("joint_angle(7) : %f", joint_id_to_rad_[7]);
	//ROS_INFO("joint_angle(8) : %f", joint_id_to_rad_[8]);

	if(joint_id_to_rad_[7]>DEG2RAD_(50))
	{
		joint_id_to_rad_[7]=DEG2RAD_(50);
	}
	else if(joint_id_to_rad_[7]<-DEG2RAD_(10))
	{
		joint_id_to_rad_[7]=-DEG2RAD_(10);
	}

	if(joint_id_to_rad_[8]>DEG2RAD_(45))
	{
		joint_id_to_rad_[8]=DEG2RAD_(45);
	}
	else if(joint_id_to_rad_[8]<-DEG2RAD_(45))
	{
		joint_id_to_rad_[8]=-DEG2RAD_(45);
	}


	if(alice_id_ == "3")
	{
		if(joint_id_to_rad_[9]>DEG2RAD_(45))
		{
			joint_id_to_rad_[9]=DEG2RAD_(45);
		}
		else if(joint_id_to_rad_[9]<-DEG2RAD_(45))
		{
			joint_id_to_rad_[9]=-DEG2RAD_(45);
		}

	}

	//ROS_INFO("-----------------------------------\n");
	//ROS_INFO("receive rad\n");

	//ROS_INFO("ID 7 Value : %f \n", joint_id_to_rad_[7]);
	//ROS_INFO("ID 8 Value : %f \n", joint_id_to_rad_[8]);

	if(alice_id_ == "3" || alice_id_ == "1")
	{
		//ROS_INFO("ID 9 Value : %f \n", joint_id_to_rad_[9]);
	}

	is_moving_state=true;

}

int TorsoModule::error_calculate(int origin_point, int object_point)
{
	return (object_point - origin_point);
}

void TorsoModule::desiredPoseHeadMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	is_moving_head_ = true;
}

void TorsoModule::detectedObjectsMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg)
{
	bool inner_flag = false;

	double error_yaw;
	double error_pitch;

	if(mode_ == 3)	inner_flag = true; // tracking

	if(!inner_flag)
	{
		return;
	}

	for(int i = 0; i < msg->length; i++)
	{
		if(!msg->data[i].name.compare("ball"))
		{
			current_x = msg->data[i].roi.x_offset + msg->data[i].roi.width/2;
			current_y = msg->data[i].roi.y_offset + msg->data[i].roi.height/2;
			ball_detected = 1;
			break;
		}
	}

	if(ball_detected)
	{
		// 추가한 부분

		//x_origin = 640 - RAD2DEG_(joint_name_to_curr_pose_[joint_id_to_name_[8]]) * 9;			// -45~45 || 21.3 	-> 30 ~ -30 || 14.2	-> 30 ~ -30 || 14.2
		//y_origin = 360 + RAD2DEG_(joint_name_to_curr_pose_[joint_id_to_name_[7]]) * 4; 			// -18~18 || 20 	-> 80 ~ -10 || 20	-> 65 ~ -25 || 20

		x_origin = 640;			// Zed Cam's Resolution
		y_origin = 360; 		// Zed Cam's Resolution

		error_yaw   = DEG2RAD_(error_calculate(x_origin,current_x)/14.2);
		error_pitch = DEG2RAD_(error_calculate(y_origin,current_y)/12);

		//ROS_INFO("current_x : %f\n", current_x);
		//ROS_INFO("current_y : %f\n", current_y);

		//ROS_INFO("curr_pos_y : %f\n", RAD2DEG_(joint_name_to_curr_pose_[joint_id_to_name_[8]]) * 14.2);
		//ROS_INFO("curr_pos_p : %f\n", RAD2DEG_(joint_name_to_curr_pose_[joint_id_to_name_[7]]) * 20);

		//ROS_INFO("x_origin : %d\n", x_origin);
		//ROS_INFO("y_origin : %d\n", y_origin);

		//ROS_INFO("error_yaw : %d\n", error_calculate(x_origin,current_x));
		//ROS_INFO("error_pitch : %d\n", error_calculate(y_origin,current_y));

		update_state = false;
	}
	else
	{
		error_yaw = 0;
		error_pitch = 0;

		update_state = false;
	}

	joint_id_to_rad_[8]=joint_name_to_curr_pose_[joint_id_to_name_[8]]-error_yaw;
	joint_id_to_rad_[7]=joint_name_to_curr_pose_[joint_id_to_name_[7]]+error_pitch;
		
	if(joint_id_to_rad_[7]>DEG2RAD_(-abs(joint_id_to_rad_[8])*0.44+60))
	{
		joint_id_to_rad_[7]= -abs(joint_id_to_rad_[8])*0.44+60;
	}
	else if(joint_id_to_rad_[7]<-DEG2RAD_(10))
	{
		joint_id_to_rad_[7]=-DEG2RAD_(10);
	}

	if(joint_id_to_rad_[8]>DEG2RAD_(45))
	{
		joint_id_to_rad_[8]=DEG2RAD_(45);
	}
	else if(joint_id_to_rad_[8]<-DEG2RAD_(45))
	{
		joint_id_to_rad_[8]=-DEG2RAD_(45);
	}	
	
	//ROS_INFO("%f", joint_id_to_rad_[8]);

	is_moving_state=true;
			
}

void TorsoModule::environmentDetectorMsgCallback(const alice_msgs::FoundObjectArray::ConstPtr& msg)
{
	/*
	int data_cnt = 0;

	for(int i = 0; i < msg->length; i++)
	{
		if(!msg->data[i].name.compare("goal"))
		{
			data_cnt ++;
			current_goal_x  = msg->data[i].pos.x;
			current_goal_y  = msg->data[i].pos.y;
		}
		if(!msg->data[i].name.compare("center"))
		{
			data_cnt ++;
			current_center_x = msg->data[i].pos.x;
			current_center_y = msg->data[i].pos.y;
		}
	}

	if(data_cnt == 2)
	{
		double theta_center = 0;
		double theta_goal = 0;
		double length_center = 0;
		double length_goal = 0;
		double sin_robot = 0;
		double cos_robot = 0;

		if(current_center_x != 0 && current_goal_x != 0)
		{
			theta_center = fabs(atan2(current_center_y,current_center_x));
			theta_goal   = fabs(atan2(current_goal_y, current_goal_x));
			length_center = sqrt(pow(current_center_x,2) + pow(current_center_y,2));
			length_goal = sqrt(pow(current_goal_x,2) + pow(current_goal_y,2));
			sin_robot = (length_goal * sin(theta_center + theta_goal)) / 4.5;
			cos_robot = sqrt(1-pow(sin_robot,2));

			current_robot_x = length_center*sin_robot;
			current_robot_y = length_center*cos_robot;

			int sign_x = 1;
			int sign_y = 1;

			if((current_goal_y - current_center_y) < 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) < 0) //case 1
			{
				sign_x = 1;
				sign_y = 1;
			}
			else if((current_goal_y - current_center_y) < 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) > 0)
			{
				sign_x = -1;
				sign_y = 1;
			}
			else if((current_goal_y - current_center_y) > 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) > 0)
			{
				sign_x = -1;
				sign_y = -1;
			}
			else if((current_goal_y - current_center_y) > 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) < 0)
			{
				sign_x = 1;
				sign_y = -1;
			}
			current_robot_x = current_robot_x * sign_x;
			current_robot_y = current_robot_y * sign_y;

			if((current_goal_x - current_center_x) != 0)
				current_robot_theta = atan2((current_goal_y - current_center_y), (current_goal_x - current_goal_x));
			else
			{
				if((current_goal_y - current_center_y) > 0)
					current_robot_theta = 90*DEGREE2RADIAN;
				else if((current_goal_y - current_center_y) < 0)
					current_robot_theta = 270*DEGREE2RADIAN;
				else
					current_robot_theta = 0;

			}
		}
		else
		{
			current_center_x = 0.01;
			current_goal_x = 0.01;

			theta_center = fabs(atan2(current_center_y,current_center_x));
			theta_goal   = fabs(atan2(current_goal_y, current_goal_x));
			length_center = sqrt(pow(current_center_x,2) + pow(current_center_y,2));
			length_goal = sqrt(pow(current_goal_x,2) + pow(current_goal_y,2));
			sin_robot = (length_goal * sin(theta_center + theta_goal)) / 4.5;
			cos_robot = sqrt(1-pow(sin_robot,2));

			current_robot_x = length_center*sin_robot;
			current_robot_y = length_center*cos_robot;

			int sign_x = 1;
			int sign_y = 1;

			if((current_goal_y - current_center_y) < 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) < 0) //case 1
			{
				sign_x = 1;
				sign_y = 1;
			}
			else if((current_goal_y - current_center_y) < 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) > 0)
			{
				sign_x = -1;
				sign_y = 1;
			}
			else if((current_goal_y - current_center_y) > 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) > 0)
			{
				sign_x = -1;
				sign_y = -1;
			}
			else if((current_goal_y - current_center_y) > 0 && (pow(length_goal,2) - pow(length_center,2) - pow(4.5,2)) < 0)
			{
				sign_x = 1;
				sign_y = -1;
			}
			current_robot_x = current_robot_x * sign_x;
			current_robot_y = current_robot_y * sign_y;

			if((current_goal_x - current_center_x) != 0)
				current_robot_theta = atan2((current_goal_y - current_center_y), (current_goal_x - current_goal_x));
			else
			{
				if((current_goal_y - current_center_y) > 0)
					current_robot_theta = 90*DEGREE2RADIAN;
				else if((current_goal_y - current_center_y) < 0)
					current_robot_theta = 270*DEGREE2RADIAN;
				else
					current_robot_theta = 0;

			}
		}
	}

	data_cnt = 0;


	robot_state_msg.x = current_robot_x;
	robot_state_msg.y = current_robot_y;
	robot_state_msg.z = current_robot_theta;

	robot_state_pub.publish(robot_state_msg);
	*/

}

void TorsoModule::headMovingMsgCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg)
{

	pre_mode_ = mode_;

  if(msg->key == "head_tracking") //head tracking
  {
    mode_ = 3;
	if(pre_mode_ != 3)
	{
		new_count_ = 1;
		is_moving_state = false;
		ROS_INFO("		Tracking Mode\n");
	}
  }
  else if(msg->key == "head_searching") //head search
  {
    mode_ = 2;
	if((pre_mode_ != 2) || (search_phase == 0))
	{
		new_count_ = 1;
		search_phase = 1;
		is_moving_state = false;
		motion_phase_init = true;
		ROS_INFO("		Searching Mode\n");
	}
  }
  else if(msg->key == "head_ball_check") //head ball check
  {
    mode_ = 5;
	if((pre_mode_ != 5) || (search_phase == 0))
	{
		new_count_ = 1;
		search_phase = 5;
		is_moving_state = false;
		motion_phase_init = true;
		ROS_INFO("		Ball_check Mode\n");
	}
  }
  else if(msg->key == "head_stop") //head stop
  {
    mode_ = 6;
	if(pre_mode_ != 6)
	{
		new_count_ = 1;
		is_moving_state = false;
		ROS_INFO("		Torso Module :: STOP !!\n");
	}
  }

}

//test
void TorsoModule::ballTestMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	current_x = msg->data[0];
	current_y = msg->data[1];
}

void TorsoModule::scanCallback(const std_msgs::Bool::ConstPtr& msg)
{

}

void TorsoModule::ballTestParamMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

}

void TorsoModule::desiredPoseWaistMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	// Test
}

void TorsoModule::walkingModuleStatusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)
{


}
	
void TorsoModule::readIDData()
{
  ros::NodeHandle nh;
  int alice_id_int  = nh.param<int>("alice_userid",0);

  //ROS_INFO("Base id: %d",alice_id_int);
  std::stringstream alice_id_stream;
  alice_id_stream << alice_id_int;
  alice_id_ = alice_id_stream.str();
}

void TorsoModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	/* subscribe topics */

	// for gui
	ros::Subscriber head_manual_sub = ros_node.subscribe("/heroehs/alice/torso_mode", 5, &TorsoModule::torsomodeCallback, this);
	ros::Subscriber manual_moving_sub	= ros_node.subscribe("heroehs/alice/manual/torso_desired_point", 5 ,&TorsoModule::manualMovingCallback, this);
	head_moving_sub = ros_node.subscribe("/alice/head_command", 5, &TorsoModule::headMovingMsgCallback, this);


	scan_cmd_sub	= ros_node.subscribe("/heroehs/alice/scan_cmd", 1, &TorsoModule::scanCallback, this);

	environment_detector_sub = ros_node.subscribe("/heroehs/environment_detector", 5, &TorsoModule::environmentDetectorMsgCallback, this);
	detected_objects_sub = ros_node.subscribe("/alice/vision/detected_objects", 5, &TorsoModule::detectedObjectsMsgCallback, this);

	// test desired pose
	head_test = ros_node.subscribe("/desired_pose_head", 5, &TorsoModule::desiredPoseHeadMsgCallback, this);
	waist_test = ros_node.subscribe("/desired_pose_waist", 5, &TorsoModule::desiredPoseWaistMsgCallback, this);

	//test ball
	//ball_test_sub = ros_node.subscribe("/ball_test", 5, &TorsoModule::ballTestMsgCallback, this);
	ball_param_sub = ros_node.subscribe("/ball_param", 5, &TorsoModule::ballTestParamMsgCallback, this);

	//walking status
	walking_module_status_sub = ros_node.subscribe("/heroehs/status", 10, &TorsoModule::walkingModuleStatusMsgCallback, this);

	/* publish topics */

	scan_done_pub	= ros_node.advertise<std_msgs::Bool>("/heroehs/alice/scan_done", 1);
	robot_state_pub = ros_node.advertise<geometry_msgs::Vector3>("/heroehs/alice/robot_state", 1);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);

	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}

bool TorsoModule::isRunning()
{
	return running_;
}

void TorsoModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{
	if (enable_ == false)
	{
		return;
	}

	if(new_count_ == 1)
	{
		update_state = true;

		new_count_++;

		if(alice_id_ == "2")
		{
			for(int i=7; i<9;i++)
			{
				result_[joint_id_to_name_[i]]->goal_position_=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
				joint_name_to_curr_pose_[joint_id_to_name_[i]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
				joint_id_to_rad_[joint_name_to_id_[joint_id_to_name_[i]]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
			}
		}
		else if(alice_id_ == "3")
		{
			for(int i=7; i<10;i++)
			{
				result_[joint_id_to_name_[i]]->goal_position_=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
				joint_name_to_curr_pose_[joint_id_to_name_[i]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
				joint_id_to_rad_[joint_name_to_id_[joint_id_to_name_[i]]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
			}
		}
	}

	if(is_moving_state==true && mode_== 1)   	// manual mode
	{
		if(alice_id_ == "2")
		{
			for(int i=7; i<9;i++)
			{
				joint_name_to_curr_pose_[joint_id_to_name_[i]] = dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
				
				//  PID gain 변경
				error_[i-7] = joint_id_to_rad_[i] - joint_name_to_curr_pose_[joint_id_to_name_[i]];
				
				if(abs(error_[i-7]) < min_error_[i-7])	error_[i-7] = min_error_[i-7];
				if(abs(error_[i-7]) > max_error_[i-7])	error_[i-7] = max_error_[i-7];
				
				if(abs(error_[i-7]) > (max_error_[i-7] / 3))	mapped_error_[i-7] = max_p_gain[i-7] - (abs(error_[i-7]) / (max_error_[i-7] -min_error_[i-7]) * (max_p_gain[i-7] - min_p_gain[i-7]) + min_p_gain[i-7]);
				else											mapped_error_[i-7] = (abs(error_[i-7]) - min_error_[i-7]) * (max_p_gain[i-7] - min_p_gain[i-7]) / (max_error_[i-7] - min_error_[i-7]) + min_p_gain[i-7];

				//ROS_INFO("curr_pose : %f des_pos : %f error %f", joint_name_to_curr_pose_[joint_id_to_name_[i]], joint_id_to_rad_[i], error_[i-7]);

				//mapped_error_[i-7] = max_p_gain[i-7] - ((abs(error_[i-7]) - min_error_[i-7]) * (min_p_gain[i-7] - 0) / (max_error_[i-7] - min_error_[i-7]) + 0);
				mapped_error_[i-7] = max_p_gain[i-7] - (abs(error_[i-7]) / (max_error_[i-7] -min_error_[i-7]) * (max_p_gain[i-7] - min_p_gain[i-7]) + min_p_gain[i-7]);

				if(mapped_error_[i-7] > max_p_gain[i-7])	mapped_error_[i-7] = max_p_gain[i-7];
				if(mapped_error_[i-7] < min_p_gain[i-7])	mapped_error_[i-7] = min_p_gain[i-7];

				//ROS_INFO("error: %f mapped_error: %f", abs(error_[0]), mapped_error_[0]);
				
				dxl_pidcontroller[joint_id_to_name_[i]]->PID_set_gains(mapped_error_[i-7],0,0.00); 
				result_[joint_id_to_name_[i]]->goal_position_=joint_name_to_curr_pose_[joint_id_to_name_[i]]
																					+ dxl_pidcontroller[joint_id_to_name_[i]]->PID_process(joint_id_to_rad_[i],joint_name_to_curr_pose_[joint_id_to_name_[i]]);
				
				
				//printf("id: %d=> %f | ",i,result_[joint_id_to_name_[i]]->goal_position_);
			}
			//printf("\n");
		}
		else if(alice_id_ == "3")
		{
			for(int i=7; i<9;i++)
			{
				joint_name_to_curr_pose_[joint_id_to_name_[i]] = dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
				
				//  PID gain 변경
				error_[i-7] = joint_id_to_rad_[i] - joint_name_to_curr_pose_[joint_id_to_name_[i]];
				
				if(abs(error_[i-7]) < min_error_[i-7])	error_[i-7] = min_error_[i-7];
				if(abs(error_[i-7]) > max_error_[i-7])	error_[i-7] = max_error_[i-7];
				
				if(abs(error_[i-7]) > (max_error_[i-7] / 3))	mapped_error_[i-7] = max_p_gain[i-7] - (abs(error_[i-7]) / (max_error_[i-7] -min_error_[i-7]) * (max_p_gain[i-7] - min_p_gain[i-7]) + min_p_gain[i-7]);
				else											mapped_error_[i-7] = (abs(error_[i-7]) - min_error_[i-7]) * (max_p_gain[i-7] - min_p_gain[i-7]) / (max_error_[i-7] - min_error_[i-7]) + min_p_gain[i-7];

				//ROS_INFO("curr_pose : %f des_pos : %f error %f", joint_name_to_curr_pose_[joint_id_to_name_[i]], joint_id_to_rad_[i], error_[i-7]);

				//mapped_error_[i-7] = max_p_gain[i-7] - ((abs(error_[i-7]) - min_error_[i-7]) * (min_p_gain[i-7] - 0) / (max_error_[i-7] - min_error_[i-7]) + 0);
				mapped_error_[i-7] = max_p_gain[i-7] - (abs(error_[i-7]) / (max_error_[i-7] -min_error_[i-7]) * (max_p_gain[i-7] - min_p_gain[i-7]) + min_p_gain[i-7]);

				if(mapped_error_[i-7] > max_p_gain[i-7])	mapped_error_[i-7] = max_p_gain[i-7];
				if(mapped_error_[i-7] < min_p_gain[i-7])	mapped_error_[i-7] = min_p_gain[i-7];

				//ROS_INFO("error: %f mapped_error: %f", abs(error_[0]), mapped_error_[0]);
				
				dxl_pidcontroller[joint_id_to_name_[i]]->PID_set_gains(mapped_error_[i-7],0,0.00); 
				result_[joint_id_to_name_[i]]->goal_position_=joint_name_to_curr_pose_[joint_id_to_name_[i]]
																					+ dxl_pidcontroller[joint_id_to_name_[i]]->PID_process(joint_id_to_rad_[i],joint_name_to_curr_pose_[joint_id_to_name_[i]]);
				
				
				//printf("id: %d=> %f | ",i,result_[joint_id_to_name_[i]]->goal_position_);
			}
			//printf("\n");
		}
	}
	else if(is_moving_state==true && mode_== 3)  // tracking mode
	{
		if(alice_id_ == "2")
		{
			for(int i=7; i<9;i++)
			{
				
				joint_name_to_curr_pose_[joint_id_to_name_[i]]=dxls[joint_id_to_name_[i]]->dxl_state_->present_position_;
				
				/*
				//  PID gain 변경
				error_[i-7] = joint_id_to_rad_[i] - joint_name_to_curr_pose_[joint_id_to_name_[i]];
				
				if(abs(error_[i-7]) < min_error_[i-7])	error_[i-7] = min_error_[i-7];
				if(abs(error_[i-7]) > max_error_[i-7])	error_[i-7] = max_error_[i-7];
				
				if(abs(error_[i-7]) > (max_error_[i-7] / 3))	mapped_error_[i-7] = max_p_gain[i-7] - (abs(error_[i-7]) / (max_error_[i-7] -min_error_[i-7]) * (max_p_gain[i-7] - min_p_gain[i-7]) + min_p_gain[i-7]);
				else											mapped_error_[i-7] = (abs(error_[i-7]) - min_error_[i-7]) * (max_p_gain[i-7] - min_p_gain[i-7]) / (max_error_[i-7] - min_error_[i-7]) + min_p_gain[i-7];

				//ROS_INFO("curr_pose : %f des_pos : %f error %f", joint_name_to_curr_pose_[joint_id_to_name_[i]], joint_id_to_rad_[i], error_[i-7]);

				//mapped_error_[i-7] = max_p_gain[i-7] - ((abs(error_[i-7]) - min_error_[i-7]) * (min_p_gain[i-7] - 0) / (max_error_[i-7] - min_error_[i-7]) + 0);
				mapped_error_[i-7] = max_p_gain[i-7] - (abs(error_[i-7]) / (max_error_[i-7] -min_error_[i-7]) * (max_p_gain[i-7] - min_p_gain[i-7]) + min_p_gain[i-7]/2);

				if(mapped_error_[i-7] > max_p_gain[i-7])	mapped_error_[i-7] = max_p_gain[i-7];
				if(mapped_error_[i-7] < min_p_gain[i-7])	mapped_error_[i-7] = min_p_gain[i-7];

				//ROS_INFO("error: %f mapped_error: %f", abs(error_[0]), mapped_error_[0]);
				

				dxl_pidcontroller[joint_id_to_name_[i]]->PID_set_gains(mapped_error_[i-7],0,0.00); 
				*/

				result_[joint_id_to_name_[i]]->goal_position_=joint_name_to_curr_pose_[joint_id_to_name_[i]]
																					+ dxl_pidcontroller[joint_id_to_name_[i]]->PID_process(joint_id_to_rad_[i],joint_name_to_curr_pose_[joint_id_to_name_[i]]);
				update_state = true;

			}
		}
	}
	else if(mode_== 2)  // searching mode
	{
		if(is_moving_state)
		{
			//ROS_INFO("into moving state!!\n");

			for(int i=7; i<9;i++)
			{
				result_[joint_id_to_name_[i]]->goal_position_= motion_trajectory[joint_id_to_name_[i]]->fifth_order_traj_gen(joint_name_to_ini_pose_state_[joint_id_to_name_[i]],
				joint_name_to_ini_pose_goal_[joint_id_to_name_[i]],0,0,0,0,0,mov_time_state);
			}
		}

		if(search_phase == 1)
		{
			if(motion_phase_init)
			{
				for(int dxl_id = 7; dxl_id < 9; dxl_id++)
				{
					joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]]	 = dxls[joint_id_to_name_[dxl_id]]->dxl_state_->present_position_;
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_pose  = joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]];
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_time  = 0;

					joint_name_to_ini_pose_goal_[joint_id_to_name_[dxl_id]]	= search_motion[search_phase - 1][dxl_id-7];
				}
				motion_phase_init = false;
				is_moving_state = true;
			}	

			if(abs(dxls[joint_id_to_name_[7]]->dxl_state_->present_position_ - search_motion[search_phase-1][0]) < motion_bias &&
			abs(dxls[joint_id_to_name_[8]]->dxl_state_->present_position_ - search_motion[search_phase-1][1]) < motion_bias)
			{
				search_phase++;
				//mov_time_state = 2.0;
				is_moving_state = false;
				new_count_ = 1;
				motion_phase_init = true;
			}
		}
		else if(search_phase == 2)
		{
			if(motion_phase_init)
			{
				for(int dxl_id = 7; dxl_id < 9; dxl_id++)
				{
					joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]]	 = dxls[joint_id_to_name_[dxl_id]]->dxl_state_->present_position_;
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_pose  = joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]];
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_time  = 0;

					joint_name_to_ini_pose_goal_[joint_id_to_name_[dxl_id]]	= search_motion[search_phase - 1][dxl_id-7];
				}
				motion_phase_init = false;
				is_moving_state = true;
			}	
			if(abs(dxls[joint_id_to_name_[7]]->dxl_state_->present_position_ - search_motion[search_phase-1][0]) < motion_bias &&
			abs(dxls[joint_id_to_name_[8]]->dxl_state_->present_position_ - search_motion[search_phase-1][1]) < motion_bias)
			{
				search_phase++;
				is_moving_state = false;
				new_count_ = 1;
				motion_phase_init = true;
			}
		}
		else if(search_phase == 3)
		{
			if(motion_phase_init)
			{
				for(int dxl_id = 7; dxl_id < 9; dxl_id++)
				{
					joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]]	 = dxls[joint_id_to_name_[dxl_id]]->dxl_state_->present_position_;
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_pose  = joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]];
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_time  = 0;

					joint_name_to_ini_pose_goal_[joint_id_to_name_[dxl_id]]	= search_motion[search_phase - 1][dxl_id-7];
				}
				motion_phase_init = false;
				is_moving_state = true;
			}	
			if(abs(dxls[joint_id_to_name_[7]]->dxl_state_->present_position_ - search_motion[search_phase-1][0]) < motion_bias &&
			abs(dxls[joint_id_to_name_[8]]->dxl_state_->present_position_ - search_motion[search_phase-1][1]) < motion_bias)
			{
				search_phase = 0;
				is_moving_state = false;
				new_count_ = 1;
				motion_phase_init = true;
				pre_mode_ = 6;
			}
		}
	}
	else if(mode_ == 5)  // ball_check mode
	{
		if(is_moving_state)
		{
			//ROS_INFO("into moving state!!\n");

			for(int i=7; i<9;i++)
			{
				result_[joint_id_to_name_[i]]->goal_position_= motion_trajectory[joint_id_to_name_[i]]->fifth_order_traj_gen(joint_name_to_ini_pose_state_[joint_id_to_name_[i]],
				joint_name_to_ini_pose_goal_[joint_id_to_name_[i]],0,0,0,0,0,mov_time_state);
			}
		}

		if(search_phase == 5)
		{
			if(motion_phase_init)
			{
				for(int dxl_id = 7; dxl_id < 9; dxl_id++)
				{
					joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]]	 = dxls[joint_id_to_name_[dxl_id]]->dxl_state_->present_position_;
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_pose  = joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]];
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_time  = 0;

					joint_name_to_ini_pose_goal_[joint_id_to_name_[dxl_id]]	= search_motion[search_phase - 1][dxl_id-7];
				}

				motion_phase_init = false;
				is_moving_state = true;
			}	

			if(abs(dxls[joint_id_to_name_[7]]->dxl_state_->present_position_ - search_motion[search_phase-1][0]) < motion_bias &&
			abs(dxls[joint_id_to_name_[8]]->dxl_state_->present_position_ - search_motion[search_phase-1][1]) < motion_bias)
			{
				search_phase++;
				is_moving_state = false;
				new_count_ = 1;
				motion_phase_init = true;
			}

			ROS_INFO("pre_pos : %f\n",dxls[joint_id_to_name_[7]]->dxl_state_->present_position_);
			ROS_INFO("search motion : %f\n",search_motion[search_phase-1][0]);
			ROS_INFO("gap : %f\n",abs(dxls[joint_id_to_name_[7]]->dxl_state_->present_position_ - search_motion[search_phase-1][0]));
			
		}
		else if(search_phase == 6)
		{
			if(motion_phase_init)
			{
				for(int dxl_id = 7; dxl_id < 9; dxl_id++)
				{
					joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]]	 = dxls[joint_id_to_name_[dxl_id]]->dxl_state_->present_position_;
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_pose  = joint_name_to_ini_pose_state_[joint_id_to_name_[dxl_id]];
					motion_trajectory[joint_id_to_name_[dxl_id]]->current_time  = 0;

					joint_name_to_ini_pose_goal_[joint_id_to_name_[dxl_id]]	= search_motion[search_phase - 1][dxl_id-7];
				}
				motion_phase_init = false;
				is_moving_state = true;
			}	
			if(abs(dxls[joint_id_to_name_[7]]->dxl_state_->present_position_ - search_motion[search_phase-1][0]) < motion_bias &&
			abs(dxls[joint_id_to_name_[8]]->dxl_state_->present_position_ - search_motion[search_phase-1][1]) < motion_bias)
			{
				search_phase = 0;
				is_moving_state = false;
				new_count_ = 1;
				motion_phase_init = true;
				pre_mode_ = 6;
			}
		}
	}
	else if(mode_ == 6)							// stop
	{
		is_moving_state = false;
	}

}

double TorsoModule::LowpassFilter(double x)
{
  f_cut = 25;
  w_cut = 2*PI*f_cut;
  tau = 1/w_cut;

  y = ( tau * pre_y + ts * x ) /(tau + ts) ;
  pre_x = x;
  pre_y = y;
  return y;
}

void TorsoModule::stop()
{
	new_count_ = 1;
	return;
}
