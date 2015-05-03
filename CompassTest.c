#pragma config(Sensor, S2,     ,               sensorI2CCustom)
#include "hitechnic-compass.h"     // Include compass sensor file

tHTMC compass;
int heading = 0;

task main() {
  initSensor(&compass, S2);
  while(true) {
  	// get compass heading
		readSensor(&compass);
		heading = compass.heading;

		eraseDisplay();
		nxtDrawLine(50, 32, cosDegrees(heading)*32 + 50, sinDegrees(heading)*32 + 32);
		nxtDrawCircle(18, 0, 64);
	}
}
