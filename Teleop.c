#pragma config(Hubs,  S1, MatrxRbtcs, none,     none,     none)
#pragma config(Hubs,  S4, MatrxRbtcs, none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CCustom)
#pragma config(Sensor, S3,     ,               sensorI2CCustom)
#pragma config(Sensor, S4,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_Matrix_S1_1, motor1,        tmotorMatrix, openLoop, encoder)
#pragma config(Motor,  mtr_Matrix_S1_2, motor2,        tmotorMatrix, openLoop, encoder)
#pragma config(Motor,  mtr_Matrix_S1_3, motor3,        tmotorMatrix, openLoop, encoder)
#pragma config(Motor,  mtr_Matrix_S1_4, motor4,        tmotorMatrix, openLoop, encoder)
#pragma config(Motor,  mtr_Matrix_S4_1, motor5,        tmotorMatrix, openLoop)
#pragma config(Motor,  mtr_Matrix_S4_2, motor6,        tmotorMatrix, openLoop)
#pragma config(Motor,  mtr_Matrix_S4_3, motor7,        tmotorMatrix, PIDControl, reversed)
#pragma config(Motor,  mtr_Matrix_S4_4, motor8,        tmotorMatrix, PIDControl)
#pragma config(Servo,  srvo_Matrix_S1_1, servo1,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S1_2, servo2,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S1_3, servo3,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S1_4, servo4,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S4_1, servo5,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S4_2, servo6,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S4_3, servo7,               tServoNone)
#pragma config(Servo,  srvo_Matrix_S4_4, servo8,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

///////////////////////////////////////////////////////////////////////////
// Teleop.c                                                              //
// Teleop program for Robot C using the OmniWheelDriver_gyro library     //
// Used by team OmegaVoltz at FTC Dutch Open 2015                        //
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

// include drivers
#include "JoystickDriver.c"        // Include file to handle the Bluetooth messages
#include "OmniWheelDriver_gyro.h"  // Include driver for the omniwheels
#include "FTCtools.h"              // Include some usefull scripts
#include "hitechnic-compass.h"     // Include compass sensor file
#include "hitechnic-irseeker-v2.h" // Include driver for the IR Seeker V2
#include "lego-light.h"            // Include driver for the light sensors
#include "lego-touch.h"            // Include driver for the touch sensors
#include "lego-ultrasound.h"       // Include driver for the ultrasound sensors
//#include "hitechnic-sensormux.h"   // Include driver for the sensor multiplexer

// define some constants
#define LIMIT 		10
#define rTch			msensor_S2_2
#define lTch			msensor_S2_3
#define liftServ  servo2
#define snrSens   msensor_S2_1
#define srvFrnt		servo2
#define srvClaw		servo3
int rMtr = motor8;
int lMtr = motor7;

#define POSITION_REST	   	0
#define POSITION_BOTTOM		1
#define POSITION_30CM			2
#define POSITION_60CM			3
#define POSITION_FREE			4

#define POS30CM		26
#define POS60CM		44
#define POSLIMIT	40

// define some variables
int x1 = 0;
int y1 = 0;

bool tglmode   = false;
bool prevtouch = false;

int liftPos   = 0;
float servPos   = 0;
int servShake = 0;
int t = 0;

bool done  = true;
int target = 0;

int liftState = POSITION_REST;

bool go        = true;
bool clawState = true;
bool prevJoy6  = false;
bool frntState = true;
bool prevJoy7  = false;

tHTMC compass;

task main() {
	OWinitialize(S3, 315, 225, 135, 45);	// initialize the omniwheel driver with the compass sensor on sensor port 2
	waitForStart();     // wait for start of tele-op phase
	eraseDisplay();

	while(true) {
		getJoystickSettings(joystick);

		// apply a dead zone to the joystick data and scale the data
		x1 = -deadZonei(joystick.joy1_x1, LIMIT)*100/127;
		y1 = deadZonei(joystick.joy1_y1, LIMIT)*100/127;

		// apply logarithmic controls
		x1 = x1*x1*sgn(x1)/100;
		y1 = y1*y1*sgn(y1)/100;

		// if the '1' button is pressed, snap the controls at x and y axis
		if(joy1Btn(1)) {
			x1 = x1*(abs(x1)>=abs(y1));
			y1 = y1*(abs(x1)<abs(y1));
		}

		if(joy1Btn(7)) {
			nMotorPIDSpeedCtrl[motor1] = mtrSpeedReg;
			nMotorPIDSpeedCtrl[motor2] = mtrSpeedReg;
			nMotorPIDSpeedCtrl[motor3] = mtrSpeedReg;
			nMotorPIDSpeedCtrl[motor4] = mtrSpeedReg;
		} else {
			nMotorPIDSpeedCtrl[motor1] = mtrNoReg;
			nMotorPIDSpeedCtrl[motor2] = mtrNoReg;
			nMotorPIDSpeedCtrl[motor3] = mtrNoReg;
			nMotorPIDSpeedCtrl[motor4] = mtrNoReg;
		}

		// set the drive direction, magnitude and turntarget
		OWsetDriveVec(x1/(joy1Btn(7)*2+1), y1/(joy1Btn(7)*2+1));
		OWsetRotationSpeed(pow(abs(deadZonei(joystick.joy1_x2, LIMIT)*100/127), 2)*sgn(joystick.joy1_x2)*ROTATION/(10000*(1+(4+8*joy1btn(7))*!joy1Btn(8))));

		// update the omniwheels with the latest compass sensor readings
		OWupdate();

		// the claw
		if(joy1Btn(6)&&(prevJoy6!=joy1Btn(6))) {
			clawState = !clawState;
		}
		prevJoy6 = joy1Btn(6);

		servo[srvClaw] = clawState*142+35;

		// the frontal servo
		if(joy2Btn(7)&&(prevJoy7!=joy2Btn(7))) {
			frntState = !frntState;
		}
		prevJoy7 = joy2Btn(7);

		servo[srvFrnt] = frntState*255;

		// do the various toggled driving modes
		if((joy1Btn(5)!=prevtouch)&&(prevtouch)) {
			tglmode = !tglmode;
			OWsetMode(CMPS_CMP*tglmode);
		}
		prevtouch = joy1Btn(5);

		// do calibration controls
		if(joystick.joy1_TopHat==2) {
			OWsetnoGyro();
		} else if (joystick.joy1_TopHat==6) {
			OWsetnoGyro();
		} else if (joystick.joy1_TopHat==0) {
			OWsetCalibration(heading+180);
		} else if (joystick.joy1_TopHat==4) {
			OWsetTurnTarget(driverOffset);
		}

		// lift control
		// standard positions
		if(joystick.joy2_TopHat==2) {
			liftState = POSITION_30CM; // go to 30 cm
		} else if (joystick.joy2_TopHat==6) {
			liftState = POSITION_30CM; // go to 30 cm
		} else if (joystick.joy2_TopHat==0) {
			liftState = POSITION_60CM; // go to 60 cm
		} else if (joystick.joy2_TopHat==4) {
			liftState = POSITION_BOTTOM; // go to ground
		}

		int dist = USreadDist(snrSens);
		eraseDisplay();
		nxtDisplayCenteredTextLine(1, "dist: %d", dist);
		nxtDisplayCenteredTextLine(2, "lPos: %d", liftPos);
		nxtDisplayCenteredTextLine(5, "sPos: %d", ((int)servPos));
		nxtDisplayCenteredTextLine(6, "nMtr: %d", nMotorEncoder[motorB]);

		if(abs(deadzonei(joystick.joy2_y1, 10))>10) {
			liftState = POSITION_FREE;
		}

		switch(liftState) {
			case POSITION_REST: { // ground resting position
				motor[rMtr] = 0;
				motor[lMtr] = 0;
			} break;

			case POSITION_BOTTOM: { // go to the ground
				motor[rMtr] = -100*(!TSreadState(lTch));
				motor[lMtr] = -100*(!TSreadState(lTch));

				if(TSreadState(lTch)) {
					liftState = POSITION_REST;
				}
			} break;

			case POSITION_30CM: { // go to the 30 cm position
				if((dist < POS30CM)||(dist==255)) {
					motor[rMtr] =  100 - 3*pow(constraini(5-abs(dist-POS30CM), 0, 5)*(!TSreadState(lTch)), 2);
					motor[lMtr] =  100 - 3*pow(constraini(5-abs(dist-POS30CM), 0, 5)*(!TSreadState(lTch)), 2);
				} else {
					motor[rMtr] = -100 + 20*constraini(5-(dist-POS30CM), 0, 5)*(!TSreadState(lTch));
					motor[lMtr] = -100 + 20*constraini(5-(dist-POS30CM), 0, 5)*(!TSreadState(lTch));
				}
			} break;

			case POSITION_60CM: {
				if((dist < POS60CM)||(dist==255)) {
					motor[rMtr] = 100 - 20*constraini(5-abs(dist-POS60CM), 0, 5)*(!TSreadState(lTch);// 100 - 4*pow(constraini(5-abs(dist-POS60CM), 0, 5)*(!TSreadState(lTch)), 2);
					motor[lMtr] = 100 - 20*constraini(5-abs(dist-POS60CM), 0, 5)*(!TSreadState(lTch);//100 - 4*pow(constraini(5-abs(dist-POS60CM), 0, 5)*(!TSreadState(lTch)), 2);
				} else {
					motor[rMtr] = -100 + 20*constraini(5-abs(dist-POS60CM), 0, 5)*(!TSreadState(lTch));
					motor[lMtr] = -100 + 20*constraini(5-abs(dist-POS60CM), 0, 5)*(!TSreadState(lTch));
				}
			} break;

			case POSITION_FREE: { // no automatic control
				int lMove = deadzonei(joystick.joy2_y1, LIMIT)*100/127;
				lMove = sgn(lMove)*lMove*lMove/100;
				if(!joy2Btn(5)) {
					if(lMove>0) {
						if((dist<POSLIMIT)||(dist==255)) {
							motor[rMtr] = lMove;
							motor[lMtr] = lMove;
						}
					} else {
						motor[rMtr] = lMove*(!TSreadState(lTch));
						motor[lMtr] = lMove*(!TSreadState(lTch));
					}
				} else {
					motor[rMtr] = lMove;
					motor[lMtr] = lMove;
				}
			} break;

			default: { // stop the motors, stay at rest position
				motor[rMtr] = 0;
				motor[lMtr] = 0;
			} break;
		}

		// balls
		if(joy2Btn(8)) { // suck in the balls
			motor[motorB] = 100;
			motor[motorC] = 100;
			t = 0;
			done = true;
			go   = true;
		} else if(joy2Btn(6)) {
			motor[motorB] = -100;
			motor[motorC] = -100;
			t = 0;
			done = true;
			go   = true;
		} else { // semi-automagic ball spewing
			if(joy2Btn(1)||joy2Btn(2)||joy2Btn(3)||joy2Btn(4)) {
				if(go) {
					if(done) {
						nMotorEncoder[motorB] = 0;
						target = 0;
						done   = false;
						t      = 10;
					}

					if(joy2Btn(1)) {
						target -= 160;
					} else if(joy2Btn(2)) {
						target -= 300;
					} else if(joy2Btn(3)) {
						target -= 470;
					} else if(joy2Btn(4)) {
						target -= 620;
					}
				}
				go = false;
			} else {
				go = true;
			}

			if(nMotorEncoder[motorB] < target) {
				done = true;
			}

			if(done) {
				if(t>0) {
					motor[motorB] = 100;
					motor[motorC] = 100;
					t--;
				} else {
					motor[motorB] = 0;
					motor[motorC] = 0;
				}
			} else {
				motor[motorB] = -100;
				motor[motorC] = -100;
			}
		}
	}
}
