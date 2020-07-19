#include "pa_PID.h"
#include "math.h"

pa_PID::pa_PID(float kp1,float ki1,float kd1)
{
    kp = kp1;
    kd = kd1;
    ki = ki1;
}
void pa_PID::setPid(float kp1,float ki1,float kd1){
    kp = kp1;
    kd = kd1;
    ki = ki1;
}
float pa_PID::calcPid(float err)
{
    
    iSum+=err;
    if(iSum>Max_iSum)iSum=Max_iSum;
    float result = this->kp * err + this->kd * (err - lastErr)+this->ki*iSum;
    lastErr=err;
    return result;
}