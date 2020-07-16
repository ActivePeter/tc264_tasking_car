#include "pa_PID.h"

pa_PID::pa_PID(float kp, float ki, float kd)
{
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
}

float pa_PID::calcPid(float err)
{
    float result = this->kp * err + this->kd * (err - lastErr);
    lastErr=err;
    return result;
}