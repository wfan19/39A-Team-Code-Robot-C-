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

void setCatapult(int speed, int time) {
	setCatapult(speed);
	wait1Msec(time);
	setCatapult(0);
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

void lockBase(int position, double k, int time) {
	int begin = nSysTime;
	while (true) {
		setDrive(-k*(SensorValue[rightQuad] - position));
		if (nSysTime - begin >= time)
			break;
	}
}

int baseError;

int lockBase(int position) {
	double kp = 1.2;
	double kd = .083;
	int der = kd*((position - SensorValue[rightQuad]) - baseError;
	setDrive(-kp*(SensorValue[rightQuad] - position) + der);
	return position - SensorValue[rightQuad];
}

void driveByEncoder(int distance) {  //distance is in **Inches**

	SensorValue[rightQuad] = 0;
	int power;
	int current = SensorValue[rightQuad];
	int target = ((distance / 12.56)* 360) + current;
	int error = target - current;
	int der = 0;
	int lastError = 0;
	float kp = 0.534;
	float kd = 0.083;

	while(abs(error) > 5) {
		current = SensorValue[rightQuad];
		error = target - current; //right quad sensor value is flipped
		der = error - lastError;
		lastError = error;
		power = error * kp + der * kd;
		setDrive(power);
		delay(20);
	}
	lockBase(current, .3, 200);
	setDrive(0);
}

void lockBaseTurn() {

	int current = SensorValue[rightQuad];
	int arcLength = current;
	int lastError = 0;
	int error = arcLength - current;
	int power = 0;
	int der = 0;
	float kp = 0.534;
	float kd = 0.083;

	int begin = nSysTime;

	while (nSysTime - begin >= 200) {
		current = SensorValue[rightQuad];
		error = arcLength - current;

		der = error - lastError;
		lastError = error;

		power = error * kp + der * kd;
		setDrive(-power, power);
		delay(20);
	}
}

void turnByEncoder(int angle){
	int current = SensorValue[rightQuad];
	int arcLength = (angle * 35.7175/360) * 360 /12.56 + current;
	int lastError = 0;
	int error = arcLength - current;
	int power = 0;
	int der = 0;
	float kp = 0.534;
	float kd = 0.083;

	while(abs(error) > 5) {
		current = SensorValue[rightQuad];
		error = arcLength - current;

		der = error - lastError;
		lastError = error;

		power = error * kp + der * kd;
		setDrive(-power * .55, power * .55);
		delay(20);
	}
	//lockBaseTurn();
	setDrive(0);
}


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

	SensorValue[rightQuad] = 0;
}


task autonomous()
{
	//----- drive forward, shoot, hit bottom flag-----
	//driveByEncoder(10);
	//delay(100);
	//setCatapult(127, 500);
	//driveByEncoder(28);
	//------end-----

	//setDrive(0);
	//driveByEncoder(10);

	turnByEncoder(90);
	//lockBaseTurn();
}

task usercontrol()
{
	// User control code here, inside the loop

	while (true)
	{
		bool lastArm = false;
		bool lastBase = false;
		bool dir = false;
		bool catRun = true;
		bool lockedBase = false;
		int currentLock;

		while (true) {

			lockedBase = newPress(Btn7U, lastBase) ? !lockedBase : lockedBase;
			lastBase = vexRT[Btn7U];

			if (!lockedBase) {
				setDrive(dir ? -deadzone(Ch2) : deadZone(Ch3), dir ? -deadZone(Ch3) : deadZone(Ch2)) ;
				currentLock = SensorValue[rightQuad];
				} else {
				baseError = lockBase(currentLock);
			}

			dir = newPress(Btn8U, lastArm) ? !dir : dir;
			setArm(buttonToPower(Btn6D, Btn8D, 127));
			lastArm = vexRT[Btn8U];

			if (catRun) {
				setCatapult(buttonToPower(Btn6U, 127));
				catRun = !vexRT[Btn7R];
				} else {
				catRun = lowerCatapult(3900);
			}

			setIntake(buttonToPower(Btn5D, Btn5U, 127));

			delay(20);
		}
	}
}
