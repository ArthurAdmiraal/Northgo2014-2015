///////////////////////////////////////////////////////////////////////////
// OmniWheelDriver.c                                                     //
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
#define DEBUG    1 // 1 for debug information, 0 for no debug information
int CMPS_CMP = 0; // 1 for compass compensated, 0 for no compass compensation

// import libraries
#include "hitechnic-compass.h"     // Include compass sensor file
#include "FTCtools.h"              // Inlcude some useful tools

// options - set as needed
float DRIVER_CAL      = 0;          // for blue, -45 for red. Changes the offset of the drive direction
int DRIVER_MOVE_OFF = -90;          // a calibration value for driving forward

// define some constants
#define ROTATION 100               // a constant used for finetuning the turn speed

// define some variables
// a public variable, so the main program can read out the heading too
int heading    = 0;
int prevhead   = 0; // the previous heading
tHTMC compass;

// public variables, so the main program can set or read out these values directly
int moveDir    = 0;
int moveSpeed  = 0;
int turnTarget = 0;

// private global variables, so all the scripts in the library can read them out
tSensors compasssensor = S2;

int motorangle[4]    = {45, 135, 225, 315};
int motorbuff[4]     = {0,0,0,0};

// some variables to control the turning
int prevdiff = 0; // the pervious difference in angles from the angle target and the last angle
int turn     = 0;
bool setTurn = false;
int setTurnSpeed = 0;
float whatdone = 1;

bool isBusy = false;

//////////////////////////////////////////////////////////////////////////////////
// OWinitialize(sensor compasssensor); ///////////////////////////////////////////
// initializes the OmniWheel library /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void OWinitialize(tSensors cs, int a1, int a2, int a3, int a4) {
	initSensor(&compass, cs);
	motorangle[0] = map360i(a1);
	motorangle[1] = map360i(a2);
	motorangle[2] = map360i(a3);
	motorangle[3] = map360i(a4);
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

	// get compass heading
	readSensor(&compass);
	heading = compass.heading;

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

	if(setTurn) {
		turn    	 = setTurnSpeed;
		setTurn 	 = false;
		turnTarget = heading+180;
		isBusy = true;
	} else if(abs(diff)>5){
		isBusy = true;
		int headdiff = abs(heading-prevhead);
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

	CMPS_CMP = !joy1Btn(8);

	if(joystick.joy1_TopHat==2) {
		DRIVER_CAL += 0.5;
	} else if (joystick.joy1_TopHat==6) {
		DRIVER_CAL -= 0.5;
	}

	// add all the calibration values to the moveDir and limit the data
	rot = moveDir - heading*CMPS_CMP + 180*CMPS_CMP + DRIVER_CAL*CMPS_CMP + DRIVER_MOVE_OFF;
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
	setTurn      = (abs(d1)>0);
	setTurnSpeed = -constraini(d1, -ROTATION, ROTATION)*10;
}

bool isDone() {
	return(!isBusy);
}
