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
		MyPID(uint16_t interval, float kP=0.0, float kI=0.0, float kD=0.0, uint16_t Imax=0); // interval in milliseconds
		MyPID(float deltaTime, float kP=0.0, float kI=0.0, float kD=0.0, uint16_t Imax=0);
		float updateController(float newError); // newError
		float updateController(float setPoint, float measured);
		void setParameters(float, float, float, uint16_t);
		void set_kP(float);
		void set_kI(float);
		void set_kD(float);
		void set_Imax(uint16_t);
		float get_kP();
		float get_kI();
		float get_kD();
		uint16_t get_Imax();
		void setInterval(uint16_t);
		void setDeltaTime(float);
		float getDeltaTime();
		void resetController();

		void setupDerivativeLowPassFilter(float cutOffFrequency); // enables drivative LPF filter and set it up
		void disableDerivativeLowPassFilter(); // disables derivative low-pass filter
		
		
		
	private:
		float lastError;
		float integral;
		float deltaTime;
		
		struct
		{
			float kP;
			float kI;
			float kD;
			uint16_t Imax;
		} params;

		bool enableDerivativeLPF_flag = false;
		LowPassFilter<float> derivativeLPF; // derivative low-pass filter
};


#endif
