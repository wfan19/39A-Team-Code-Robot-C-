#pragma config(Sensor, in1,    catPot,         sensorPotentiometer)
#pragma config(Sensor, dgtl9,  leftQuad,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, rightQuad,      sensorQuadEncoder)
#pragma config(Motor,  port1,           intake,        tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           pDriveLF,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           pDriveLM,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           pDriveLR,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           catR,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           catL,          tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           pDriveRF,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           pDriveRM,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           pDriveRR,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          arm,           tmotorVex393_HBridge, openLoop)

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

void setDrive(int leftSpeed, int rightSpeed) {
	motor[pDriveLF] = motor[pDriveLM] = motor[pDriveLR] = leftSpeed;
	motor[pDriveRF] = motor[pDriveRM] = motor[pDriveRR] = rightSpeed;
}

void setDrive(int speed) {
	setDrive(speed, speed);
}

void setDrive() {
	setDrive(127);
}

void setCatapult(int speed) {
	motor[catR] = motor[catL] = speed;
}

void setCatapult() {
	setCatapult(127);
}

void setIntake(int speed) {
	motor[intake] = speed;
}

void setIntake() {
	setIntake(127);
}

void setArm(int speed) {
	motor[arm] = speed;
}

void setArm() {
	setArm(127);
}

int buttonToPower(int downButton, int upButton, int power) {
	return vexRT[downButton] ? -power : vexRT[upButton] ? power : 0;
}

int buttonToPower(int button, int power) {
	return vexRT[button] ? power : 0;
}

int deadZone(int channel){
	return abs(vexRT[channel]) >= 10 ? vexRT[channel] : 0;
}

bool newPress(int channel, bool lastInput) {
	return lastInput ? false : vexRT[channel];
}

bool lowerCatapult(int position) {
	if (SensorValue[catPot] < position) {
		setCatapult(127);
		return false;
	} else {
		return true;
	}
}

void lockBase(int position) {
	double k = .45;
	setDrive(k*(SensorValue[rightQuad] - position));
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks
	// running between Autonomous and Driver controlled modes. You will need to
	// manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
	// ..........................................................................
	// Insert user code here.
	// ..........................................................................

	// Remove this function call once you have "real" code.
	AutonomousCodePlaceholderForTesting();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
	// User control code here, inside the loop

	while (true)
	{
		bool lastArm = false;
		bool lastBase = false;
		bool dir = false;
		bool catRun = true;
		float turtle = 1;
		bool lockedBase = false;
		int currentLock;

		while (true) {
			turtle = vexRT[Btn5U] ? 0.5 : 1;

			lockedBase = newPress(Btn7U, lastBase) ? !lockedBase : lockedBase;
			lastBase = vexRT[Btn7U];

			if (!lockedBase) {
				setDrive(deadZone(Ch3) * turtle, deadZone(Ch2) * turtle) ;
				currentLock = SensorValue[rightQuad];
				} else {
				lockBase(currentLock);
			}

			dir = newPress(Btn8U, lastArm) ? !dir : dir;
			setArm(buttonToPower(Btn6D, Btn8D, 127*(dir ? 1 : -1)));
			lastArm = vexRT[Btn8U];

			if (catRun) {
				setCatapult(buttonToPower(Btn6U, 127));
				catRun = !vexRT[Btn7R];
				} else {
				catRun = lowerCatapult(3700);
			}

			setIntake(buttonToPower(Btn5D, Btn5U, 127));

			delay(20);
		}
	}
}
