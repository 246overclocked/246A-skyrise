/*
 * x_drive.h
 *
 *  Created on: Oct 17, 2014
 *      Author: Oliver
 */

#ifndef X_DRIVE_H_
#define X_DRIVE_H_

#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "vex.h"

typedef enum {
	Disabled,
	PID,
	Components,
	FieldCentric,
	Tank
}x_driveMode;

typedef struct {
	tVexAnalogPin	gyroPort;
	tVexMotor		lfMotor;
	tVexMotor		rfMotor;
	tVexMotor		lbMotor;
	tVexMotor		rbMotor;

	x_driveMode		driveMode;

	int16_t			gyro;
	int16_t			shouldResetGyro;
	int16_t			shouldResetCenter;
	int16_t			gyroSetPoint = 0;
	int16_t			gyroOffset = 0;
	int16_t			gyroError;
	int16_t			manualTurnPwr;
	int16_t			turnPwr;
	int16_t			maxTurnPwr = 27;
	int16_t			UPwr;
	int16_t			VPwr;
	int16_t			XPwr;
	int16_t			YPwr;

	tCtlIndex		RUCtl;
	tCtlIndex		LUCtl;
	tCtlIndex		RVCtl;
	tCtlIndex		LVCtl;

}xDrive;

void	x_driveInit(tVexMotor lfMotor, tVexMotor rfMotor, tVexMotor lbMotor, tVexMotor rbMotor, tVexAnalogPin gyro);
void	setXDriveControls(tCtlIndex RUCtl, tCtlIndex LUCtl, tCtlIndex RVCtl, tCtlIndex LVCtl, );
void	setXDriveMode(x_driveMode mode);
void	beginTankControl()

#endif /* X_DRIVE_H_ */
