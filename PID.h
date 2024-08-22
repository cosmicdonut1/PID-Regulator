#ifndef _PID_H_
#define _PID_H_

#include <Arduino.h>

struct PIDdata
{
    float       P_Factor;
    float       I_Factor;
    float     D_Factor;
    float       dZone;
    float       controlMax;
    float       controlMin;
    float       lastError;
    float       integratedError;
    unsigned long   previousPIDTime;
    float       target;
};

void initPID(float p, float i, float d, float dzone, float max_out, float min_out, float target, struct PIDdata *PIDparameters)
{
    PIDparameters->P_Factor     = p;
    PIDparameters->I_Factor     = i;
    PIDparameters->D_Factor     = d;
    PIDparameters->dZone        = dzone;
    PIDparameters->controlMax     = max_out;
    PIDparameters->controlMin     = min_out;
    PIDparameters->integratedError  = 0;
    PIDparameters->lastError      = 0;
    PIDparameters->previousPIDTime  = micros();
    PIDparameters->target       = target;
}

float updatePID(float currentPosition, struct PIDdata *PIDparameters)
{
    unsigned long now = micros();

    float deltaPIDTime = (now - PIDparameters->previousPIDTime) / 1000000.0;

    float error = PIDparameters->target - currentPosition;

    if (abs(error) < PIDparameters->dZone)
    {
        error = 0;
    }

    PIDparameters->integratedError += (PIDparameters->I_Factor * error * deltaPIDTime);
    PIDparameters->integratedError = constrain(PIDparameters->integratedError, PIDparameters->controlMin, PIDparameters->controlMax);
	
    float p_component = PIDparameters->P_Factor * error;
    float i_component = PIDparameters->integratedError;
    float d_component = PIDparameters->D_Factor * ((error - PIDparameters->lastError) / deltaPIDTime);

    float control = p_component + PIDparameters->integratedError + d_component;

    control = constrain(control, PIDparameters->controlMin, PIDparameters->controlMax);

    PIDparameters->lastError = error;
    PIDparameters->previousPIDTime = now;

    return control;
}

void resetIntegralError(struct PIDdata *PIDparameters)
{
    PIDparameters->integratedError    = 0;
    PIDparameters->lastError        = 0;
    PIDparameters->previousPIDTime    = micros();
}

void setTarget(float target, struct PIDdata *PIDparameters)
{
    PIDparameters->target       = target;
}
#endif
