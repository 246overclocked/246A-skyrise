/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     vexuser.c                                                    */
/*    Author:     James Pearman                                                */
/*    Created:    7 May 2013                                                   */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00  XX XXX 2013 - Initial release                         */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include <stdlib.h>
#include <math.h>

#include "ch.h"  		// needs for all ChibiOS programs
#include "hal.h" 		// hardware abstraction layer header
#include "vex.h"		// vex library header

#include "vexgyro.h"
#include "smartmotor.h"
#include "robotc_glue.h"


// Digi IO configuration
static  vexDigiCfg  dConfig[kVexDigital_Num] = {
        { kVexDigital_1,    kVexSensorQuadEncoder,  kVexConfigQuadEnc1,       kVexQuadEncoder_1 },
        { kVexDigital_2,    kVexSensorQuadEncoder,  kVexConfigQuadEnc2,       kVexQuadEncoder_1 },
        { kVexDigital_3,    kVexSensorQuadEncoder,  kVexConfigQuadEnc1,       kVexQuadEncoder_2 },
        { kVexDigital_4,    kVexSensorQuadEncoder,  kVexConfigQuadEnc2,       kVexQuadEncoder_2 },
        { kVexDigital_5,    kVexSensorQuadEncoder,  kVexConfigQuadEnc1,       kVexQuadEncoder_3 },
        { kVexDigital_6,    kVexSensorQuadEncoder,  kVexConfigQuadEnc2,       kVexQuadEncoder_3 },
        { kVexDigital_7,    kVexSensorQuadEncoder,  kVexConfigQuadEnc1,       kVexQuadEncoder_4 },
        { kVexDigital_8,    kVexSensorQuadEncoder,  kVexConfigQuadEnc2,       kVexQuadEncoder_4 },
        { kVexDigital_9,    kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_10,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_11,   kVexSensorDigitalInput,  kVexConfigInput,       0 },
        { kVexDigital_12,   kVexSensorDigitalInput,  kVexConfigInput,       0 }
};

static  vexMotorCfg mConfig[kVexMotorNum] = {
        { kVexMotor_1,      kVexMotorUndefined,           kVexMotorNormal,       kVexSensorNone,         0 },
        { kVexMotor_2,      kVexMotor393T,      kVexMotorNormal ,       kVexSensorIME,        kImeChannel_1 }, //Front Left Drive (PE_A)
        { kVexMotor_3,      kVexMotor393T,      kVexMotorNormal,       kVexSensorQuadEncoder,        kImeChannel_2 }, //Front Right Drive
        { kVexMotor_4,      kVexMotor393T,      kVexMotorNormal,       kVexSensorQuadEncoder,        kVexQuadEncoder_3 }, //Back Left Drive
        { kVexMotor_5,      kVexMotor393T,      kVexMotorNormal,       kVexSensorQuadEncoder,        kVexQuadEncoder_4 }, //Back Right Drive (PE_B)
        { kVexMotor_6,      kVexMotor393T,      kVexMotorNormal,       kVexSensorNone,        0 },	//Right Conveyor Motors (PE_C)
        { kVexMotor_7,      kVexMotor393T,      kVexMotorNormal,       kVexSensorNone,        0 }, //Left Conveyor Motors
        { kVexMotor_8,      kVexMotor393S,      kVexMotorNormal,       kVexSensorNone,        0 }, //Top Elevator (PE_D)
        { kVexMotor_9,      kVexMotor393S,      kVexMotorNormal,       kVexSensorNone,        0 }, //Bottom Elevator
        { kVexMotor_10,     kVexMotorUndefined,           kVexMotorNormal,       kVexSensorNone,         0 }
};


/*-----------------------------------------------------------------------------*/
/** @brief      User setup                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  The digital and motor ports can (should) be configured here.
 */
void
vexUserSetup()
{
	vexDigitalConfigure( dConfig, DIG_CONFIG_SIZE( dConfig ) );
	vexMotorConfigure( mConfig, MOT_CONFIG_SIZE( mConfig ) );
	vexMotorDirectionSet(kVexMotor_2, kVexMotorReversed);
}

/*-----------------------------------------------------------------------------*/
/** @brief      User initialize                                                */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This function is called after all setup is complete and communication has
 *  been established with the master processor.
 *  Start other tasks and initialize user variables here
 */
void
vexUserInit()
{
	SmartMotorsInit();
	SmartMotorPtcMonitorDisable();
	SmartMotorCurrentMonitorDisable();
	SmartMotorsAddPowerExtender(kVexMotor_2, kVexMotor_4);
	SmartMotorSetPowerExpanderStatusPort(kVexAnalog_2);
	SmartMotorRun();
	vexGyroInit(kVexAnalog_1);
}

int
cappedPowerValue( int value, int cap)
{
	if ( value <= cap && value >= - cap )
	{
		return value;
	}
	else if ( value > cap )
	{
		return cap;
	}
	else
	{
		return - cap;
	}
}

/*-----------------------------------------------------------------------------*/
/** @brief      Autonomous                                                     */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the autonomous period is started
 */
msg_t
vexAutonomous( void *arg )
{
    (void)arg;

    // Must call this
    vexTaskRegister("auton");

    while(1)
        {
        // Don't hog cpu
        vexSleep( 25 );
        }

    return (msg_t)0;
}

#define PI 3.1415926

msg_t
FCDriveTask( void )
{
	int16_t		gyro;
	int16_t		shouldResetGyro;
	int16_t		shouldResetCenter;
	int16_t		gyroSetPoint = 0;
	int16_t		gyroOffset = 0;
	int16_t		gyroError;
	int16_t		manualTurnPwr;
	int16_t		turnPwr;
	int16_t		maxTurnPwr = 27;
	int16_t		UPwr;
	int16_t		VPwr;
	int16_t		XPwr;
	int16_t		YPwr;
	float		cosine;
	float		sine;

	vexTaskRegister("driveTask");

	while(1)
	{
		gyro = vexGyroGet()-gyroOffset; //Get raw gyroscope value

		//Display gyroscope value
		vexLcdPrintf(VEX_LCD_DISPLAY_1,VEX_LCD_LINE_1,"GYRO: %d", gyro);

		//Manually set turning position
		shouldResetGyro = vexControllerGet (Btn5D);
		if (shouldResetGyro==1)
		{
			gyroSetPoint = gyro; //Update setpoint to current position
		}

		//Manually reset gyro and turning position
		shouldResetCenter = vexControllerGet (Btn5U);
		if(shouldResetCenter==1)
		{
			gyroOffset = vexGyroGet(); //Reset gyro position
			gyro = vexGyroGet()-gyroOffset; //Read new gyro position
			gyroSetPoint = 0;
		}

		//Calculate distance from set turning position and adjust power proportionally
		gyroError = (gyro-gyroSetPoint)*.15;

		//Manual turning
		manualTurnPwr = vexControllerGet (Ch4);
		if (abs(manualTurnPwr)>10)//Manual control if over threshold
		{
			turnPwr = (int16_t) floor ( manualTurnPwr * maxTurnPwr / 127) ; //Adjust to keep turnPwr at less than 27
		}
		else	//Automatic turning to last set point
		{
			turnPwr = cappedPowerValue( gyroError, maxTurnPwr ) ; //Cap gyro error for turn
		}

		//Read raw joystick axis before conversion
		UPwr = vexControllerGet (Ch1);
		VPwr = vexControllerGet (Ch2);

		//Find trig for conversion
		cosine	=  cosf ( PI / 180 *  (gyro * .10 + 45) ) ;
		sine	=  sinf ( PI / 180 *  (gyro * .10 + 45) ) ;

		//Trinonometric axis rotation
		XPwr =(int16_t)floor( ( UPwr * cosine + VPwr * sine ) * ( 127 - maxTurnPwr ) / -127);
		YPwr =(int16_t)floor( ( VPwr * cosine - UPwr * sine ) * ( 127 - maxTurnPwr ) / 127);

		//Output calculated values to motors
		SetMotor ( kVexMotor_2 , - XPwr - turnPwr);
		SetMotor ( kVexMotor_3 , YPwr - turnPwr);
		SetMotor ( kVexMotor_5 , - XPwr + turnPwr);
		SetMotor ( kVexMotor_4 , YPwr + turnPwr);

		// Don't hog cpu
		vexSleep( 25 );
	}
	return(msg_t)0;
}

msg_t
TankStyleXDriveTask( void )
{
	int16_t		LUPwr, LVPwr, LXPwr, LYPwr, RUPwr, RVPwr, RXPwr, RYPwr;
	float		cosine, sine;
	vexTaskRegister("tankDriveTask");

	while (1){
		//Read raw joystick axis before conversion
		RUPwr = vexControllerGet (Ch1);
		RVPwr = vexControllerGet (Ch2);
		LUPwr = vexControllerGet (Ch1);
		LVPwr = vexControllerGet (Ch3);

		//Find trig for conversion
		cosine =  cosf ( PI / 180 * 45 ) ;
		sine =  sinf ( PI / 180 * 45 ) ;

		//Trinonometric axis rotation
		RXPwr =(int16_t)floor( ( RUPwr * cosine + RVPwr * sine ) * -1 );
		RYPwr =(int16_t)floor( RVPwr * cosine - RUPwr * sine );
		LXPwr =(int16_t)floor( ( LUPwr * cosine + LVPwr * sine ) * -1);
		LYPwr =(int16_t)floor( LVPwr * cosine - LUPwr * sine );

		//Output calculated values to motors
		SetMotor ( kVexMotor_2 , - RXPwr);
		SetMotor ( kVexMotor_3 , RYPwr);
		SetMotor ( kVexMotor_5 , - LXPwr);
		SetMotor ( kVexMotor_4 , LYPwr);

		// Don't hog cpu
		vexSleep( 25 );
	}
	return(msg_t)0;
}

msg_t
LiftControlTask( void )
{
	int16_t liftU, liftD, liftPwr ;
	vexTaskRegister("liftControlTask");
	while (1){
		liftU = vexControllerGet (Btn6U) ;
		liftD = vexControllerGet (Btn6D) ;
		liftPwr = ( liftU - liftD ) * 127 ;
		SetMotor ( kVexMotor_8 , liftPwr ) ;
		SetMotor ( kVexMotor_9 , - liftPwr ) ;
		// Don't hog cpu
		vexSleep( 25 );
	}
	return(msg_t)0;
}

msg_t
ConveyorControlTask( void )
{
	int16_t conveyorPwr;
	vexTaskRegister("conveyorControlTask");
	while (1){

		conveyorPwr = vexControllerGet (Ch4Xmtr2) ;
		SetMotor ( kVexMotor_6 , conveyorPwr ) ;
		SetMotor ( kVexMotor_7 , - conveyorPwr ) ;
		if (vexAdcGet(kVexAnalog_7)<2650&&vexAdcGet(kVexAnalog_7)>2000){
			conveyorPwr = 0 ;
			SetMotor ( kVexMotor_6 , conveyorPwr ) ;
			SetMotor ( kVexMotor_7 , - conveyorPwr ) ;
			vexSleep(900);
			conveyorPwr = vexControllerGet (Ch4Xmtr2) ;
			SetMotor ( kVexMotor_6 , conveyorPwr ) ;
			SetMotor ( kVexMotor_7 , - conveyorPwr ) ;
			vexSleep(200);
		}
		// Don't hog cpu
		vexSleep( 10 );
	}
	return(msg_t)0;
}

/*-----------------------------------------------------------------------------*/
/** @brief      Driver control                                                 */
/*-----------------------------------------------------------------------------*/
/** @details
 *  This thread is started when the driver control period is started
 */
msg_t
vexOperator( void *arg )
{

	(void)arg;

	// Must call this
	vexTaskRegister("operator");

	if (vexAdcGet( kVexAnalog_8 )>200){
		StartTaskWithPriority( TankStyleXDriveTask, NORMALPRIO );
	}
	else {
		StartTaskWithPriority( FCDriveTask, NORMALPRIO );
	}

	StartTaskWithPriority( LiftControlTask, NORMALPRIO );

	StartTaskWithPriority( ConveyorControlTask, NORMALPRIO );

	// Run until asked to terminate
	while(!chThdShouldTerminate())
		{

		// Don't hog cpu
		vexSleep( 25 );
		}

	return (msg_t)0;
}



