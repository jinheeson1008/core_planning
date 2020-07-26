/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "op_pid_controller_core.h"


namespace op_pid_controller_ns
{

MotionController::MotionController()
{
	bNewCurrentPos = false;
	bNewTrajectory = false;
	bNewBehaviorState = false;
	bNewVehicleStatus = false;
	m_bAutoCalibrate = false;
	m_bVelocityCalibrate = false;

	ros::NodeHandle _nh;

	UpdateControlParams(_nh);

	pub_ControlInfoRviz = _nh.advertise<visualization_msgs::MarkerArray>("op_pid_controller_points", 1);
	pub_VehicleCommand = _nh.advertise<autoware_msgs::VehicleCmd>("op_controller_cmd", 1);
	// Control Topics Sections
	//----------------------------
	sub_current_pose = _nh.subscribe("/current_pose", 1,	&MotionController::callbackGetCurrentPose, this);
	int bVelSource = 1;
	_nh.getParam("/op_common_params/velocitySource", bVelSource);
	std::string velocity_topic;
	if(bVelSource == 0)
	{
		_nh.getParam("/op_common_params/vel_odom_topic", velocity_topic);
		sub_robot_odom = _nh.subscribe(velocity_topic, 1, &MotionController::callbackGetRobotOdom, this);
	}
	else if(bVelSource == 1)
	{
		_nh.getParam("/op_common_params/vel_curr_topic", velocity_topic);
		sub_current_velocity = _nh.subscribe(velocity_topic, 1, &MotionController::callbackGetAutowareStatus, this);
	}
	else if(bVelSource == 2)
	{
		_nh.getParam("/op_common_params/vel_can_topic", velocity_topic);
		sub_can_info = _nh.subscribe(velocity_topic, 1, &MotionController::callbackGetCANInfo, this);
	}
	else if(bVelSource == 3)
	{
		_nh.getParam("/op_common_params/vehicle_status_topic", velocity_topic);
		sub_vehicle_status = _nh.subscribe(velocity_topic, 1, &MotionController::callbackGetVehicleStatus, this);
	}

	//----------------------------

  	sub_behavior_state = _nh.subscribe("/op_current_behavior",	1, &MotionController::callbackGetBehaviorState, 	this);
  	sub_current_trajectory = _nh.subscribe("/op_local_selected_trajectory", 1,	&MotionController::callbackGetCurrentTrajectory, this);


  	m_Controller.Init(m_ControlParams, m_CarInfo, true, m_bAutoCalibrate);

  	if(m_bVelocityCalibrate)
  	{
  		double max_v = 0;
		_nh.getParam("/op_pid_controller/calibrate_max_velocity", max_v );
  		m_TargetTestSpeeds.push_back({0,max_v});
  		m_TargetTestSpeeds.push_back({max_v,0});
		m_TargetTestSpeeds.push_back({0,max_v});
  		m_TargetTestSpeeds.push_back({max_v,0});
  	}

	std::cout << "OP PID controller initialized successfully " << std::endl;
}

MotionController::~MotionController()
{
}

void MotionController::UpdateControlParams(ros::NodeHandle& nh)
{
	nh.getParam("/op_common_params/width", m_CarInfo.width);
	nh.getParam("/op_common_params/length", m_CarInfo.length);
	nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	nh.getParam("/op_common_params/maxWheelAngle", m_CarInfo.max_wheel_angle);
	nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
	nh.getParam("/op_common_params/maxVelocity", m_CarInfo.max_speed_forward);
	nh.getParam("/op_common_params/minVelocity", m_CarInfo.min_speed_forward);
	nh.getParam("/op_common_params/steeringDelay", m_ControlParams.SteeringDelay);
	nh.getParam("/op_common_params/minPursuiteDistance", m_ControlParams.minPursuiteDistance );

	nh.getParam("/op_common_params/experimentName", m_Controller.m_ExperimentFolderName);
	if(m_Controller.m_ExperimentFolderName.size() > 0)
	{
		if(m_Controller.m_ExperimentFolderName.at(m_Controller.m_ExperimentFolderName.size()-1) != '/')
		{
			m_Controller.m_ExperimentFolderName.push_back('/');
		}
	}

	int steering_mode = 0;
	int drive_mode = 0;
	nh.getParam("/op_pid_controller/steer_mode", steering_mode );
	nh.getParam("/op_pid_controller/drive_mode", drive_mode);
	//nh.getParam("/op_pid_controller/auto_calibration_mode", m_bAutoCalibrate ); // disabled for now
	nh.getParam("/op_pid_controller/manual_velocity_calibration", m_bVelocityCalibrate );


	nh.getParam("/op_pid_controller/avg_engine_brake_accel", m_ControlParams.avg_engine_brake_accel );
	nh.getParam("/op_pid_controller/min_follow_safe_distance", m_ControlParams.min_safe_follow_distance);

	nh.getParam("/op_pid_controller/lowpass_steer_cutoff", m_ControlParams.LowpassSteerCutoff );
	nh.getParam("/op_pid_controller/control_frequency", m_ControlParams.ControlFrequency );

	nh.getParam("/op_pid_controller/max_steer_value", m_CarInfo.max_steer_value );

	nh.getParam("/op_pid_controller/max_steer_torque", m_CarInfo.max_steer_torque );
	nh.getParam("/op_pid_controller/min_steer_torque", m_CarInfo.min_steer_torque );

	nh.getParam("/op_pid_controller/max_accel_value", m_CarInfo.max_accel_value );
	nh.getParam("/op_pid_controller/max_brake_value", m_CarInfo.max_brake_value );

	nh.getParam("/op_pid_controller/steerGainKP", m_ControlParams.Steering_Gain.kP );
	nh.getParam("/op_pid_controller/steerGainKI", m_ControlParams.Steering_Gain.kI );
	nh.getParam("/op_pid_controller/steerGainKD", m_ControlParams.Steering_Gain.kD );

	nh.getParam("/op_pid_controller/velocityGainKP", m_ControlParams.Velocity_Gain.kP );
	nh.getParam("/op_pid_controller/velocityGainKI", m_ControlParams.Velocity_Gain.kI );
	nh.getParam("/op_pid_controller/velocityGainKD", m_ControlParams.Velocity_Gain.kD );

	nh.getParam("/op_pid_controller/accelGainKP", m_ControlParams.Accel_Gain.kP );
	nh.getParam("/op_pid_controller/accelGainKI", m_ControlParams.Accel_Gain.kI );
	nh.getParam("/op_pid_controller/accelGainKD", m_ControlParams.Accel_Gain.kD );

	nh.getParam("/op_pid_controller/brakeGainKP", m_ControlParams.Brake_Gain.kP );
	nh.getParam("/op_pid_controller/brakeGainKI", m_ControlParams.Brake_Gain.kI );
	nh.getParam("/op_pid_controller/brakeGainKD", m_ControlParams.Brake_Gain.kD );

}

void MotionController::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
//	std::cout << "Current Pose, X: " << m_CurrentPos.pos.x << ", Y: " << m_CurrentPos.pos.y << std::endl;
	bNewCurrentPos = true;
}

void MotionController::callbackGetAutowareStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;

	if(fabs(m_CurrentPos.v) > 0.1)
	{
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/m_CurrentPos.v);
	}

	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);

//	std::cout << "Vehicle Status, Speed: " << m_VehicleStatus.speed << ", Steer Angle: " << m_VehicleStatus.steer << ", Original Angular: " << msg->twist.angular.z << std::endl;
	bNewVehicleStatus = true;
}

void MotionController::callbackGetCANInfo(const autoware_can_msgs::CANInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_CurrentPos.v = m_VehicleStatus.speed;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_wheel_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);

//	std::cout << "CAN Info, Speed: " << m_VehicleStatus.speed << ", Steer Angle: " << m_VehicleStatus.steer << ", Original Angle: " << msg->angle << std::endl;
	bNewVehicleStatus = true;
}

void MotionController::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed ;
	if(msg->twist.twist.linear.x != 0)
	{
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	}
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);

//	std::cout << "Odomerty, Speed: " << m_VehicleStatus.speed << ", Steer Angle: " << m_VehicleStatus.steer << ", Original Angle: " << msg->twist.twist.angular.z << std::endl;

	bNewVehicleStatus = true;
}

void MotionController::callbackGetVehicleStatus(const autoware_msgs::VehicleStatusConstPtr & msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = -msg->angle*DEG2RAD;
	m_CurrentPos.v = m_VehicleStatus.speed;

//	std::cout << "Vehicle Real Status, Speed: " << m_VehicleStatus.speed << ", Steer Angle: " << m_VehicleStatus.steer << ", Steermode: " << msg->steeringmode << ", Org angle: " << msg->angle <<  std::endl;
}

void MotionController::callbackGetBehaviorState(const autoware_msgs::WaypointConstPtr& msg )
{
	m_CurrentBehavior = PlannerHNS::ROSHelpers::ConvertAutowareWaypointToBehaviorState(*msg);
	//std::cout << "Receive Behavior State : " << m_CurrentBehavior.state << ", Target Speed: " << m_CurrentBehavior.maxVelocity << ", StopD: " << m_CurrentBehavior.stopDistance << ", FollowD: " << m_CurrentBehavior.followDistance << std::endl;
	bNewBehaviorState = true;
}

void MotionController::callbackGetCurrentTrajectory(const autoware_msgs::LaneConstPtr &msg)
{
	m_FollowingTrajectory.clear();
	PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(*msg, m_FollowingTrajectory);
	std::cout << "Receive new Trajectory From Behavior Selector : " << msg->waypoints.size() << std::endl;
	bNewTrajectory = true;
}

void MotionController::displayFollowingInfo(const PlannerHNS::WayPoint& perp_pose, const PlannerHNS::WayPoint& follow_pose, visualization_msgs::MarkerArray& points_markers)
{

  visualization_msgs::Marker m1, m3;
  m1.header.frame_id = "map";
  m1.header.stamp = ros::Time();
  m1.ns = "curr_simu_pose";
  m1.type = visualization_msgs::Marker::ARROW;
  m1.action = visualization_msgs::Marker::ADD;
  m1.pose.position.x = perp_pose.pos.x;
  m1.pose.position.y = perp_pose.pos.y;
  m1.pose.position.z = perp_pose.pos.z;
  m1.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(perp_pose.pos.a));
  std_msgs::ColorRGBA green;
  green.a = 1.0;
  green.b = 0.0;
  green.r = 0.0;
  green.g = 1.0;
  m1.color = green;
  m1.scale.x = 1.8;
  m1.scale.y = 0.5;
  m1.scale.z = 0.5;
  m1.frame_locked = true;
  points_markers.markers.push_back(m1);

  m3.header.frame_id = "map";
  m3.header.stamp = ros::Time();
  m3.ns = "follow_pose";
  m3.type = visualization_msgs::Marker::SPHERE;
  m3.action = visualization_msgs::Marker::ADD;
  m3.pose.position.x = follow_pose.pos.x;
  m3.pose.position.y = follow_pose.pos.y;
  m3.pose.position.z = follow_pose.pos.z;
  m3.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(follow_pose.pos.a));
  std_msgs::ColorRGBA red;
  red.a = 1.0;
  red.b = 0.0;
  red.r = 1.0;
  red.g = 0.0;
  m3.color = red;
  m3.scale.x = 0.7;
  m3.scale.y = 0.7;
  m3.scale.z = 0.7;
  m3.frame_locked = true;
  points_markers.markers.push_back(m3);
}

void MotionController::MainLoop()
{
	int freq = m_ControlParams.ControlFrequency;
	ros::Rate loop_rate(freq);

	struct timespec dt_timer;
	UtilityHNS::UtilityH::GetTickCount(dt_timer);
	double dt = 1.0/(double)freq;
	double avg_dt = dt;

	while (ros::ok())
	{
		ros::spinOnce();

		dt = UtilityHNS::UtilityH::GetTimeDiffNow(dt_timer);
		UtilityHNS::UtilityH::GetTickCount(dt_timer);

		dt_list.push_back(dt);
		if(dt_list.size() > freq)
		{
			double dt_sum = 0;
			for(auto& step_dt: dt_list)
			{
				dt_sum += step_dt;
			}
			avg_dt = dt_sum / dt_list.size();
			dt_list.erase(dt_list.begin()+0);
		}

		if(m_bVelocityCalibrate && m_FollowingTrajectory.size() > 0)
		{
			if(m_TargetTestSpeeds.size() > 0)
			{
				m_CurrentBehavior.maxVelocity = m_TargetTestSpeeds.front().second;

				if((m_TargetTestSpeeds.front().second > m_TargetTestSpeeds.front().first && m_VehicleStatus.speed >= m_TargetTestSpeeds.front().second) ||
						(m_TargetTestSpeeds.front().second < m_TargetTestSpeeds.front().first && m_VehicleStatus.speed <= m_TargetTestSpeeds.front().second))
				{
					m_Controller.ResetLogTime(m_VehicleStatus.speed,m_TargetTestSpeeds.front().first);
					m_TargetTestSpeeds.erase(m_TargetTestSpeeds.begin()+0);
				}
			}
			else
			{
				m_CurrentBehavior.maxVelocity = 0;
			}

			if(m_VehicleStatus.speed == 0.0)
			{
				m_Controller.ResetLogTime(0,0);
			}
		}

		m_TargetStatus = m_Controller.DoOneStep(avg_dt, m_CurrentBehavior, m_FollowingTrajectory, m_CurrentPos, m_VehicleStatus, bNewTrajectory);

//		std::cout << "Curr Steer Angle: " << m_VehicleStatus.steer << ", Target Steer Angle: " << m_TargetStatus.steer_torque << ", Max Steer   : " << m_CarInfo.max_wheel_angle << std::endl;
//		std::cout << "Curr Velocity   : " << m_VehicleStatus.speed << ", Target Accel   : " << m_TargetStatus.accel_stroke << ", Max Velocity: " << m_CurrentBehavior.maxVelocity << std::endl;
//		std::cout << "Curr Velocity   : " << m_VehicleStatus.speed << ", Target Brake   : " << m_TargetStatus.brake_stroke << ", Max Velocity: " << m_CurrentBehavior.maxVelocity << std::endl;

		autoware_msgs::VehicleCmd cmd;
		cmd.steer_cmd.steer = m_TargetStatus.steer_torque;
		cmd.accel_cmd.accel = m_TargetStatus.accel_stroke;
		cmd.brake_cmd.brake = m_TargetStatus.brake_stroke;

		pub_VehicleCommand.publish(cmd);

		visualization_msgs::MarkerArray points_markers;
		displayFollowingInfo(m_Controller.m_PerpendicularPoint, m_Controller.m_FollowMePoint, points_markers);
		pub_ControlInfoRviz.publish(points_markers);

		loop_rate.sleep();
	}
}

}
