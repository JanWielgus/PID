/*
  MyPID.h - PID library
  Created by Jan Wielgus, October 6, 2018.
*/

#ifndef _MYPID_h
#define _MYPID_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


class MyPID
{
	public:
		MyPID(uint16_t interval, float kP=0.0, float kI=0.0, float kD=0.0, uint16_t Imax=0); // interval in milliseconds
		MyPID(float deltaTime, float kP=0.0, float kI=0.0, float kD=0.0, uint16_t Imax=0);
		float updateController(float); // newError
		float updateController(float, float); // setPoint, newValue
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
		
		
		
	private:
		//static double deltaT; // [s]
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
		
};


#endif

