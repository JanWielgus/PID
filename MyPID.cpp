/*
  MyPID.cpp - PID library
  Created by Jan Wielgus, October 6, 2018.
*/

#include <MyPID.h>

MyPID::MyPID(uint16_t interval, float kP, float kI, float kD, uint16_t Imax)
{
	this->params.kP = kP;
	this->params.kI = kI;
	this->params.kD = kD;
	this->params.Imax = Imax;
	this->deltaTime = interval * 0.001;
	
	// defaults:
	lastError = 0;
	integral = 0;
}


MyPID::MyPID(float deltaTime, float kP, float kI, float kD, uint16_t Imax)
{
	this->params.kP = kP;
	this->params.kI = kI;
	this->params.kD = kD;
	this->params.Imax = Imax;
	this->deltaTime = deltaTime;
	
	// defaults:
	lastError = 0;
	integral = 0;
}


float MyPID::updateController(float setPoint, float measured);
{
	return updateController(setPoint - measured);
}


float MyPID::updateController(float newError)
{
	// I term
	integral += (newError * params.kI) * deltaTime;
	integral = constrain(integral, -params.Imax, params.Imax); // Anti wind-up term
	
	// D term
	float derivative = (newError - lastError) / deltaTime;
	lastError = newError;

	if (enableDerivativeLPF_flag)
		derivative = derivativeLPF.update(derivative); // Low-pass filter
	
	return newError*params.kP + integral + derivative*params.kD;
}


void MyPID::setParameters(float kP, float kI, float kD, uint16_t Imax)
{
	set_kP(kP);
	set_kI(kI);
	set_kD(kD);
	set_Imax(Imax);
}

void MyPID::set_kP(float kP)
{
	this->params.kP = kP;
}

void MyPID::set_kI(float kI)
{
	this->params.kI = kI;
}

void MyPID::set_kD(float kD)
{
	this->params.kD = kD;
}

void MyPID:: set_Imax(uint16_t imax)
{
	this->params.Imax = imax;
}

float MyPID::get_kP()
{
	return params.kP;
}

float MyPID::get_kI()
{
	return params.kI;
}

float MyPID::get_kD()
{
	return params.kD;
}

uint16_t MyPID::get_Imax()
{
	return params.Imax;
}


void MyPID::setInterval(uint16_t interval)
{
	this->deltaTime = interval * 0.001;
}


void MyPID::setDeltaTime(float deltaTIme)
{
	this->deltaTime = deltaTIme;
}


float MyPID::getDeltaTime()
{
	return deltaTime;
}


void MyPID::resetController()
{
	lastError = 0;
	integral = 0;
}


void MyPID::setDerivativeLowPassFilterParams(float cutOffFrequency)
{
	enableDerivativeLPF_flag = true;
	derivativeLPF.reconfigureFilter(cutOffFrequency, this->deltaTime);
}





