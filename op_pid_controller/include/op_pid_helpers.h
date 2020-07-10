/*
 * op_pid_helpers.h
 *
 *  Created on: July 02, 2020
 *      Author: Hatem Darweesh
 */

#ifndef OP_PID_HELPERS_H
#define OP_PID_HELPERS_H

#include <math.h>
#include <string>

namespace op_pid_controller_ns
{

enum STATUS_TYPE{VEHICLE_STATUS, ROBOT_STATUS, SIMULATION_STATUS};

class ControlCommandParams
{
public:
	STATUS_TYPE statusSource;
	bool bTorqueMode; // true -> torque and stroke mode, false -> angle and velocity mode
	bool bAngleMode;
	bool bVelocityMode;
	bool bEnableLogs;
	bool bCalibration;

	ControlCommandParams()
	{
		statusSource = SIMULATION_STATUS;
		bTorqueMode = false;
		bAngleMode = true;
		bVelocityMode = true;
		bEnableLogs = false;
		bCalibration = false;
	}
};

} /* namespace op_pid_controller_ns */

#endif /* OP_PID_HELPERS_H */
