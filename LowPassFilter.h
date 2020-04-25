/*
	LowPassFilter library by overlord1123 under GPL-2.0 license
	https://github.com/overlord1123/LowPassFilter
*/

#ifndef _LowPassFilter_h_
#define _LowPassFilter_h_

#include "arduino.h"


class LowPassFilter
{
public:
	//constructors
	LowPassFilter();
	LowPassFilter(float iCutOffFrequency, float iDeltaTime);
	//functions
	float update(float input);
	float update(float input, float deltaTime, float cutoffFrequency);
	//get and configure funtions
	float getOutput() const{return output;}
	void reconfigureFilter(float deltaTime, float cutoffFrequency);
private:
	float output;
	float ePow;
};

#endif //_LowPassFilter_h_
