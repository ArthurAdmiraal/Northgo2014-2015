///////////////////////////////////////////////////////////////////////////
// FTCtools.c                                                            //
// some usefull scripts for the FTC                                      //
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

// import usefull libraries
#include "hitechnic-compass.h"     // Include compass sensor file
#include "hitechnic-irseeker-v2.h" // Include driver for the IR Seeker V2
#include "lego-light.h"            // Include driver for the light sensors
#include "lego-touch.h"            // Include driver for the touch sensors

// define some constants
#define BUTTONS 10                // Number of monitored buttons

// scripts used to constrain values
float constrainf(float x, float a, float b) {
	if(x<a){return(a);}
	if(x>b){return(b);}
	return(x);
}

int constraini(int x, int a, int b) {
	if(x<a){return(a);}
	if(x>b){return(b);}
	return(x);
}

// scripts used to map the data in the 0 to 360 range
float map360f(float a) {
	while(a>360.0){a -= 360.0;}
	while(a<0.0){a += 360.0;}
	return(a);
}

int map360i(int a) {
	while(a>360){a -= 360;}
	while(a<0){a += 360;}
	return(a);
}

// scripts used to add a 'dead zone' to the data
float deadZonef(float a, float lim) {
	return(a * ((a < lim) || (a > lim)) );
}

int deadZonei(int a, int lim) {
	return(a * ((a < lim) || (a > lim)) );
}

bool togglebuff[BUTTONS];

// script used for a toggle variable
bool doToggle(bool toggle, bool button, int buttonnum) {
	bool b        = toggle;
	int togglenum = constraini(buttonnum, 0, BUTTONS);

	if(button&&(button!=togglebuff[buttonnum])) {
		b = !toggle;
	}

	togglebuff[togglenum] = button;

	return(b);
}

int max4(int a, int b, int c, int d) {
	return(max2(max3(a,b,c),d));
}