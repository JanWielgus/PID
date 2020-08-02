/*
  MyPID.cpp - PID library
  Created by Jan Wielgus, October 6, 2018.
*/

#include <PID.h>


MyPID::MyPID(float deltaTime, float kP, float kI, float kD, uint16_t Imax)
{
	setGains(kP, kI, kD, Imax);
	setDeltaTime(deltaTime);
	reset();
}


float MyPID::update(float setpoint, float measurement)
{
	return update(setpoint - measurement);
}


float MyPID::update(float newError)
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


void MyPID::setGains(float kP, float kI, float kD, uint16_t Imax)
{
	set_kP(kP);
	set_kI(kI);
	set_kD(kD);
	set_Imax(Imax);
}

void MyPID::set_kP(float kP)
{
	this->kP = kP;
}

void MyPID::set_kI(float kI)
{
	this->kI = kI;
}

void MyPID::set_kD(float kD)
{
	this->kD = kD;
}

void MyPID:: set_Imax(uint16_t imax)
{
	this->Imax = imax;
}

float MyPID::get_kP()
{
	return kP;
}

float MyPID::get_kI()
{
	return kI;
}

float MyPID::get_kD()
{
	return kD;
}

uint16_t MyPID::get_Imax()
{
	return Imax;
}


void MyPID::setDeltaTime(float deltaTime)
{
	this->deltaTime = deltaTime;
}


float MyPID::getDeltaTime()
{
	return deltaTime;
}


void MyPID::reset()
{
	lastError = 0;
	integral = 0;
}


void MyPID::setupDerivativeLowPassFilter(float cutOffFrequency)
{
	enableDerivativeLPF_flag = true;
	derivativeLPF.reconfigureFilter(cutOffFrequency, this->deltaTime);
}


void MyPID::disableDerivativeLowPassFilter()
{
	enableDerivativeLPF_flag = false;
}
