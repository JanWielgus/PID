/**
 * @file PID.h
 * @author Jan Wielgus
 * @brief My PID controller library
 * @date 2018-10-06 edited 2020-08-02
 */

#ifndef _PID_h
#define _PID_h


class PID
{
public:
	/**
	 * @brief Construct new PID object.
	 * @param deltaTime Time between next update() executions [in seconds]
	 * @param kP Proportional gain
	 * @param kI Integral gain
	 * @param kD Derivative gain
	 * @param imax maximum (minimum) value of integral term
	 * @param cutOffFreq derivative low-pass filter cut-off frequency [Hz] (0 to disable)
	 */
	PID(float deltaTime, float kP=0.f, float kI=0.f, float kD=0.f, float imax=0.f, float cutOffFreq_Hz = 0.f);

	/**
	 * @brief Updates controller.
	 * @param setpoint Value that have to be hold
	 * @param measurement Measured value
	 * @return New controller value
	 */
	float update(float setpoint, float measurement);

	/**
	 * @brief Updates controller.
	 * @param newError setpoint value - measurement
	 * @return New controller value
	 */
	float update(float newError);

	/**
	 * @brief Set gains for all controller parts.
	 * @param kP Proportional gain
	 * @param kI Integral gain
	 * @param kD Derivative gain
	 * @param imax (+/-) maximum value of integral term
	 * @param cutOffFreq_Hz cut-off frequency of derivative low-pass filter [Hz]
	 * (0 to disable).
	 */
	void setGains(float kP, float kI, float kD, float imax, float cutOffFilter_Hz = 0.f);

	void set_kP(float kP);
	void set_kI(float kI);
	void set_kD(float kD);
	void set_Imax(float imax);

	/**
	 * @brief Set cut-off frequency of derivative low-pass filter [Hz]
	 * (0 to disable).
	 */
	void set_cutOffFreq(float cutOffFreq_Hz);

	float get_kP();
	float get_kI();
	float get_kD();
	float get_Imax();

	/**
	 * @brief Change delta time
	 * @param deltaTime time between next update() executions [in seconds]
	 */
	void setDeltaTime(float deltaTime);

	/**
	 * @return Delta time in seconds
	 */
	float getDeltaTime();

	/**
	 * @brief Resets controller (do not reset gain values).
	 */
	void reset();
	
	
private:
	float deltaTime;
	float lastError;
	float integral;

	// D Low-Pass Filter:
	float derivative;
	float lpf_ePow = 0.f;
	
	// PID parameters:
	float kP;
	float kI;
	float kD;
	float imax;
};


#endif
