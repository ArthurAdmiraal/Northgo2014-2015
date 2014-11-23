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

// import libraries
#include "hitechnic-compass.h"     // Include compass sensor file
#include "FTCtools.h"              // Inlcude some useful tools

// options - set as needed
int DRIVER_CAL      = 45;          // for blue, -45 for red. Changes the offset of the drive direction
int DRIVER_MOVE_OFF = 45;          // a calibration value for driving forward

// define some constants
#define ROTATION 100               // a constant used for finetuning the turn speed

// define some variables
// a public variable, so the main program can read out the heading too
int heading    = 0;
tHTMC compass;

// public variables, so the main program can set or read out these values directly
int moveDir    = 0;
int moveSpeed  = 0;
int turnTarget = 0;

// private global variables, so all the scripts in the library can read them out
tSensors compasssensor = S2;

int motorangle[4]    = {45, 135, 225, 315};
int motorbuff[4]     = {0,0,0,0};

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

	// add all the calibration values to the moveDir and limit the data
	rot = moveDir - heading + DRIVER_CAL + DRIVER_MOVE_OFF;
	rot = map360i(rot);

	// set the values to the motor buffers
	for(int i = 0; i<4; i++) {
		motorbuff[i]  = moveSpeed*(cosDegrees(motorangle[i])*cosDegrees(rot)  + sinDegrees(motorangle[i])*sinDegrees(rot));
		motorbuff[i] += diff*ROTATION/180;
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

	// print some debug information
	eraseDisplay();
	nxtDisplayCenteredTextLine(line1, "head: %d", heading);
	nxtDisplayCenteredTextLine(line2, "diff: %d", diff);
	nxtDisplayCenteredTextLine(line3, "m1: %d", motor[motor1]);
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