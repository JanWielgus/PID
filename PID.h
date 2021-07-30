/**
 * @file PID.h
 * @author Jan Wielgus
 * @brief My PID controller library
 * @date 2018-10-06 edited 2020-08-02
 * 
 */

#ifndef _PID_h
#define _PID_h

#include <LowPassFilter.h> // https://github.com/ColyberCompany/FilteringLibraries

#ifdef ARDUINO
    #include <Arduino.h>
#endif



class PID
{
public:
	/**
	 * @brief Construct new PID object.
	 * 
	 * @param deltaTime Time between next update() executions [in seconds]
	 * @param kP Proportional gain
	 * @param kI Integral gain
	 * @param kD Derivative gain
	 * @param Imax (+/-) maximum value of integral term
	 */
	PID(float deltaTime, float kP=0.0f, float kI=0.0f, float kD=0.0f, uint16_t Imax=0);

	/**
	 * @brief Updates controller.
	 * 
	 * @param setpoint Value that have to be hold
	 * @param measurement Measured value
	 * @return New controller value
	 */
	float update(float setpoint, float measurement);

	/**
	 * @brief Updates controller.
	 * 
	 * @param newError setpoint value - measurement
	 * @return New controller value
	 */
	float update(float newError);

	/**
	 * @brief Set gains for all controller parts
	 * 
	 * @param kP Proportional gain
	 * @param kI Integral gain
	 * @param kD Derivative gain
	 * @param Imax (+/-) maximum value of integral term
	 */
	void setGains(float kP, float kI, float kD, uint16_t Imax);

	void set_kP(float kP);
	void set_kI(float kI);
	void set_kD(float kD);
	void set_Imax(uint16_t imax);

	float get_kP();
	float get_kI();
	float get_kD();
	uint16_t get_Imax();

	/**
	 * @brief Change delta time
	 * 
	 * @param deltaTime time between next update() executions [in seconds]
	 */
	void setDeltaTime(float deltaTime);

	/**
	 * @return Delta time in seconds
	 */
	float getDeltaTime();

	/**
	 * @brief Resets controller
	 */
	void reset();

	/**
	 * @brief Enables derivative low-pass filter and set its parameters
	 * 
	 * @param cutOffFrequency Cut-off frequency [in Hz]
	 */
	void setupDerivativeLowPassFilter(float cutOffFrequency);

	/**
	 * @brief Disable derivative low-pass filter
	 */
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
