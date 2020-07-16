#ifndef __pa_PID_H_
#define __pa_PID_H_

class pa_PID
{
public:
    pa_PID(float kp,float ki,float kd);
    float calcPid(float err);
private:
    float kp;
    float ki;
    float kd;
    float lastErr=0;
};

#endif