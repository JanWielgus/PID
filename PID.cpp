/*
  MyPID.cpp - PID library
  Created by Jan Wielgus, October 6, 2018.
*/

#include <PID.h>
#include <math.h>

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif


PID::PID(float deltaTime, float kP, float kI, float kD, float imax, float cutOffFreq_Hz)
{
	setDeltaTime(deltaTime);
	setGains(kP, kI, kD, imax, cutOffFreq_Hz);
	reset();
}


float PID::update(float setpoint, float measurement)
{
	return update(setpoint - measurement);
}


float PID::update(float newError)
{
	// I term
	integral += (newError * kI) * deltaTime;
	integral = constrain(integral, -imax, imax); // Anti wind-up term
	
	// D term
	float newDerivative = (newError - lastError) / deltaTime;
	if (lpf_ePow == 0.f)
		derivative = newDerivative;
	else
		derivative += (newDerivative - derivative) * lpf_ePow;

	lastError = newError;
	
	return newError * kP + integral + derivative * kD;
}


void PID::setGains(float kP, float kI, float kD, float imax, float cutOffFreq_Hz)
{
	setGains(kP, kI, kD, imax);
	set_derivCutoffFreq(cutOffFreq_Hz);
}

void PID::setGains(float kP, float kI, float kD, float imax)
{
	set_kP(kP);
	set_kI(kI);
	set_kD(kD);
	set_Imax(imax);
}

void PID::set_kP(float kP)
{
	this->kP = kP;
}

void PID::set_kI(float kI)
{
	this->kI = kI;
}

void PID::set_kD(float kD)
{
	this->kD = kD;
}

void PID::set_Imax(float imax)
{
	this->imax = imax;
}

void PID::set_derivCutoffFreq(float cutOffFreq_Hz)
{
	if (cutOffFreq_Hz <= 0)
		lpf_ePow = 0;
	else
		lpf_ePow = 1 - exp(-deltaTime * 2 * M_PI * cutOffFreq_Hz);
}

float PID::get_kP()
{
	return kP;
}

float PID::get_kI()
{
	return kI;
}

float PID::get_kD()
{
	return kD;
}

float PID::get_Imax()
{
	return imax;
}


void PID::setDeltaTime(float deltaTime)
{
	this->deltaTime = deltaTime;
}


float PID::getDeltaTime()
{
	return deltaTime;
}


void PID::reset()
{
	lastError = 0;
	integral = 0;
	derivative = 0;
	lpf_ePow = 0;
}
