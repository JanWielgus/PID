/*
  MyPID.cpp - PID library
  Created by Jan Wielgus, October 6, 2018.
*/

#include <PID.h>


PID::PID(float deltaTime, float kP, float kI, float kD, uint16_t Imax)
{
	setGains(kP, kI, kD, Imax);
	setDeltaTime(deltaTime);
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
	integral = constrain(integral, -Imax, Imax); // Anti wind-up term
	
	// D term
	float derivative = (newError - lastError) / deltaTime;
	lastError = newError;

	// D term low-pass filter
	if (enableDerivativeLPF_flag)
		derivative = derivativeLPF.update(derivative); // Low-pass filter
	
	return newError * kP + integral + derivative * kD;
}


void PID::setGains(float kP, float kI, float kD, uint16_t Imax)
{
	set_kP(kP);
	set_kI(kI);
	set_kD(kD);
	set_Imax(Imax);
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

void PID::set_Imax(uint16_t imax)
{
	this->Imax = imax;
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

uint16_t PID::get_Imax()
{
	return Imax;
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
}


void PID::setupDerivativeLowPassFilter(float cutOffFrequency)
{
	enableDerivativeLPF_flag = true;
	derivativeLPF.reconfigureFilter(cutOffFrequency, this->deltaTime);
}


void PID::disableDerivativeLowPassFilter()
{
	enableDerivativeLPF_flag = false;
}
