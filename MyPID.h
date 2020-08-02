/*
  MyPID.h - PID library
  Created by Jan Wielgus, October 6, 2018.
*/

#ifndef _MYPID_h
#define _MYPID_h

#include "arduino.h"
#include <LowPassFilter.h>



class MyPID
{
public:
	MyPID(float deltaTime, float kP=0.0f, float kI=0.0f, float kD=0.0f, uint16_t Imax=0);
	float updateController(float setPoint, float measurement);
	float updateController(float newError);
	void setParameters(float kP, float kI, float kD, uint16_t Imax);
	void set_kP(float kP);
	void set_kI(float kI);
	void set_kD(float kD);
	void set_Imax(uint16_t imax);
	float get_kP();
	float get_kI();
	float get_kD();
	uint16_t get_Imax();
	void setDeltaTime(float deltaTime);
	float getDeltaTime();
	void reset();

	void setupDerivativeLowPassFilter(float cutOffFrequency); // enables drivative LPF filter and set it up
	void disableDerivativeLowPassFilter(); // disables derivative low-pass filter
	
	
	
private:
	float lastError;
	float integral;
	float deltaTime;
	
	// PID parameters:
	float kP;
	float kI;
	float kD;
	uint16_t Imax;

	// Derivative filter parts:
	bool enableDerivativeLPF_flag = false;
	LowPassFilter<float> derivativeLPF; // derivative low-pass filter
};


#endif
