/**
 * @file PIDAssess.h
 * @author your name (you@domain.com)
 * @brief Quality of PID controller (current settings of PID gains).
 * @date 2022-05-09
 */

#ifndef PIDASSESS_h
#define PIDASSESS_h

#include "PID.h"
#include <stdint.h>


/**
 * @brief Quality of PID controller (current settings of PID gains).
 */
class PIDAssess
{
    PID* pid = nullptr;
    float assessment;

    // helper:
    uint32_t samplesAmt = 0;
    float errorIntegral;

public:
    PIDAssess(PID* pid)
    {
        setPIDController(pid);
    }

    /**
     * @brief Update PID controller assessment.
     * Should be called after each PID controller update.
     * Return new assessment value (smaller value is better).
     */
    float updateAssessment()
    {
        if (!pid)
            return;
        
        samplesAmt++;
        float error = pid->lastError;

        errorIntegral += error;
        // TODO: include oscilations (how?)

        assessment = (errorIntegral / samplesAmt); // + other components
        return assessment;
    }

    /**
     * @brief Get current PID controller working assessment (smaller value is better).
     */
    float getCurrentAssessment()
    {
        return assessment;
    }

    /**
     * @brief Get how many samples were used in assessment.
     */
    float getSamplesAmt()
    {
        return samplesAmt;
    }

    /**
     * @brief Set PID controller to assess and reset assessment.
     * @param pid Pointer to PID controller to assess.
     */
    void setPIDController(PID* pid)
    {
        this->pid = pid;
        reset();
    }

    /**
     * @brief Restarts assessing (do not reset assessed PID controller).
     */
    void reset()
    {
        assessment = 0;
        errorIntegral = 0;
    }
};


#endif
