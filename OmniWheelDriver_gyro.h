///////////////////////////////////////////////////////////////////////////
// OmniWheelDriver_gyro.c                                                //
// An easy-to-use driver for driving omni-wheels                         //
// Copyright (C) 2014  Arthur Admiraal                                   //
//                                                                       //
// This program is free software: you can redistribute it and/or modify  //
// it under the terms of the GNU General Public License as published by  //
// the Free Software Foundation, either version 3 of the License, or     //
// (at your option) any later version.                                   //
//                                                                       //
// This program is distributed in the hope that it will be useful,       //
// but WITHOUT ANY WARRANTY; without even the implied warranty of        //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         //
// GNU General Public License for more details.                          //
//                                                                       //
// You should have received a copy of the GNU General Public License     //
// along with this program.  If not, see <http://www.gnu.org/licenses/>. //
///////////////////////////////////////////////////////////////////////////

// compile options
#define DEBUG    0 // 1 for debug information, 0 for no debug information
#define CMPS_CMP 1

// import libraries
#include "hitechnic-gyro.h"        // Include gyro sensor file
#include "hitechnic-compass.h"     // Include compass sensor file
#include "FTCtools.h"              // Inlcude some useful tools

// options - set as needed
int DRIVER_MOVE_OFF = -90;          // a calibration value for driving forward

// define some constants
#define ROTATION   100              // a constant used for finetuning the turn speed
#define CMPS_PORT	 msensor_S2_2

// define some variables
// a public variable, so the main program can read out the heading too
int heading    = 0;
int prevhead   = 0; // the previous heading
tHTGYRO gyroSensor;

// public variables, so the main program can set or read out these values directly
int moveDir    = 0;
int moveSpeed  = 0;
int turnTarget = 0;

// private global variables, so all the scripts in the library can read them out
tSensors gyroPort = S2;

int motorangle[4]    = {45, 135, 225, 315};
int motorbuff[4]     = {0,0,0,0};

// some variables to control the turning
int prevdiff = 0; // the pervious difference in angles from the angle target and the last angle
int turn     = 0;
bool setTurn = false;
int setTurnSpeed = 0;
float whatdone = 1;
float driverOffset = 0; // the driver standing offset

float prevTime  = 0;
float prevTime2 = 0;

int turnSpeed = 0;

int   gyroCal   = 0;
float gyroAngle = 0;

float predict  = 0;
float timediff = 0;

int OWmode = 0;

bool isBusy = false;
bool compassCompensation = true;

int prevTurnSpeed = 0;

int prevCmpHead = 0;
int cmpHead     = 0;
bool turning = false;

//////////////////////////////////////////////////////////////////////////////////
// task updateGyro() /////////////////////////////////////////////////////////////
// updates the angle /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
task updateGyro() {
	while(true) {
		hogCPU();
			readSensor(&gyroSensor);
			turnSpeed  = gyroSensor.rotation - gyroCal;
			turnSpeed  = turnSpeed*(abs(turnSpeed)>3);
			gyroAngle += ((float)turnSpeed) * ((float)((prevTime2=nPgmTime) - prevTime)-0.12) / 1000.0;
		releaseCPU();
		prevTime      = prevTime2;
		prevTurnSpeed = turnSpeed;
		wait1Msec(4);	// wait, so this process doesn't clog the CPU
	}
}

//////////////////////////////////////////////////////////////////////////////////
// OWsetnoGyro(); ////////////////////////////////////////////////////////////////
// resets gyro ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWsetnoGyro() {
	readSensor(&gyroSensor);
	gyroCal = gyroSensor.rotation;
}

///////////////////////////////////////////////////
// OWinitialize(sensor compasssensor); ///////////////////////////////////////////
// initializes the OmniWheel library /////////////////////////////////////////////
////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void OWinitialize(tSensors cs, int a1, int a2, int a3, int a4) {
	gyroPort      = cs;
	initSensor(&gyroSensor, gyroPort);
	readSensor(&gyroSensor);
	gyroCal       = gyroSensor.rotation;
	startTask(updateGyro);
	motorangle[0] = map360i(a1);
	motorangle[1] = map360i(a2);
	motorangle[2] = map360i(a3);
	motorangle[3] = map360i(a4);
	clearTimer(T4);
}

//////////////////////////////////////////////////////////////////////////////////
// OWupdate(); ///////////////////////////////////////////////////////////////////
// updates the motors with the drive data ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWupdate() {
	float factor  = 1.0;
	float highest = 0.0;
	int diff      =   0;
	int rot       =   0;

	int localTurnSpeed = turnSpeed/60;

	// get gyro heading for calculations
	heading = gyroAngle;

	// find the difference in angles with the direction target
	diff = turnTarget-heading;

	// do some calculations on the data
	diff = map360i(diff);
	diff = 180-diff;
	diff = deadZonei(diff, 10);

	// calculate what value to add to the motor to turn the robot smoothly
	// reverse turning when shooting past the target
	if(sgn(diff)!=sgn(prevdiff)) {
		turn = abs(turn)*sgn(diff);
	}

	switch(OWmode) {
		case 0: { // driver controlled turning
			turn    	 = setTurnSpeed;
			turnTarget = heading+180;
			isBusy     = true;
			OWmode     = 1;
			clearTimer(T3);
		} break;

		case 1: { // wait till turning has stopped (takes some time due to inertia) to avoid transient response
			turn       = 0;
			turnTarget = heading+180;
			isBusy     = true;

			if((abs(localTurnSpeed)<=0)&&(time1[T3]>200)) {
				OWmode = 2;
			}
		} break;

		case 2: { // do closed-loop control system
			if(abs(diff)>5){
				isBusy = true;
				int headdiff = abs(localTurnSpeed);
				if((headdiff*10)>(10+pow(diff,2)/180)) {
					turn = constraini(abs(turn)-constraini(1+1800/abs(diff),0,abs(turn)/5), 0, ROTATION*10)*sgn(diff+0.001);
				} else if((headdiff*10)<(10+pow(diff,2)/180)) {
					turn = constraini(abs(turn)+1+abs(diff)/18, 0, ROTATION*10)*sgn(diff+0.001);
				}

				if(abs(diff)<3) {
					turn = 0;
					isBusy = false;
				}
			} else {
				isBusy = false;
			}
		} break;
	}

	// add all the calibration values to the moveDir and limit the data
	rot = moveDir - heading*compassCompensation + 180*compassCompensation + driverOffset*compassCompensation + DRIVER_MOVE_OFF;
	rot = map360i(rot);

	// set the values to the motor buffers
	for(int i = 0; i<4; i++) {
		motorbuff[i]  = moveSpeed*(cosDegrees(motorangle[i])*cosDegrees(rot)  + sinDegrees(motorangle[i])*sinDegrees(rot));
		motorbuff[i] += 10*sgn(turn+0.001)*(abs(turn)>10)+turn/10;
	}

	// 'limit' the data
	highest = max4(motorbuff[0], motorbuff[1], motorbuff[2], motorbuff[3]);

	if(highest>100.0) {
		factor = 100.0/highest;
	} else {
		factor = 1.0;
	}

	// set the values to the motors
	motor[motor1] = motorbuff[0]*factor;
	motor[motor2] = motorbuff[1]*factor;
	motor[motor4] = motorbuff[2]*factor;
	motor[motor3] = motorbuff[3]*factor;

	prevhead = heading;
	prevdiff = diff;

	#if DEBUG
		// print some debug information
		eraseDisplay();
		nxtDisplayCenteredTextLine(1, "head: %d", heading);
		nxtDisplayCenteredTextLine(2, "diff: %d", diff);
		nxtDisplayCenteredTextLine(3, "m1: %d", motor[motor1]);
		nxtDisplayCenteredTextLine(4, "turn: %d", turn);
		nxtDisplayCenteredTextLine(5, "trsp: %d", localTurnSpeed);
		nxtDisplayCenteredTextLine(6, "OWmode: %d", OWmode);
	#endif
}

//////////////////////////////////////////////////////////////////////////////////
// OWsetDriveVec(); //////////////////////////////////////////////////////////////
// sets the drivedata ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWsetDriveVec(float x1, float y1) {
	// do calculations on the data
	moveSpeed = constrainf(sqrt(pow(x1, 2) + pow(y1, 2)), -100.0, 100.0);
	moveDir   = radiansToDegrees(atan2(y1, x1));
}

//////////////////////////////////////////////////////////////////////////////////
// OWsetDrive(); /////////////////////////////////////////////////////////////////
// sets the drivedata ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWsetDrive(int ms, int md) {
	moveSpeed = constraini(ms, -100, 100);
	moveDir   = map360i(md);
}

//////////////////////////////////////////////////////////////////////////////////
// OWsetTurnTarget(); ////////////////////////////////////////////////////////////
// sets the direction target /////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWsetTurnTarget(int d1) {
	turnTarget = map360i(d1);
}

//////////////////////////////////////////////////////////////////////////////////
// OWsetTurnTargetRelative(); ////////////////////////////////////////////////////
// sets the direction target relative to the current direction ///////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWsetTurnTargetRelative(int d1) {
	turnTarget = map360i(turnTarget + d1);
}

//////////////////////////////////////////////////////////////////////////////////
// OWsetRotationSpeed(); /////////////////////////////////////////////////////////
// sets the direction target relative to the current direction ///////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWsetRotationSpeed(int d1) {
	//setTurn    = (abs(d1)>0);
	if(abs(d1)>0) {
		OWmode = 0;
	}
	setTurnSpeed = -constraini(d1, -ROTATION, ROTATION)*10;
}

//////////////////////////////////////////////////////////////////////////////////
// OWisDone(); ///////////////////////////////////////////////////////////////////
// returns whether the robot is done turning /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
bool OWisDone() {
	return(!isBusy);
}

//////////////////////////////////////////////////////////////////////////////////
// OWsetmode(int mode); //////////////////////////////////////////////////////////
// sets various modes fpr the OW driver library //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWsetMode(int mode) {
	compassCompensation = mode&0b1;
}

//////////////////////////////////////////////////////////////////////////////////
// OWadjustCalibration(float adjust); ////////////////////////////////////////////
// adjust the driver standing offset by adjust ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWadjustCalibration(float adj) {
	driverOffset += adj;
}

//////////////////////////////////////////////////////////////////////////////////
// OWsetCalibration(int cal); ////////////////////////////////////////////////////
// adjust the driver standing offset by adjust ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWsetCalibration(int cal) {
	driverOffset = heading+180;
}
