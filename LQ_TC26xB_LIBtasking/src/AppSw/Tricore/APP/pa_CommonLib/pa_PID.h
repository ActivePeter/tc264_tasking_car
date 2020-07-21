#ifndef __pa_PID_H_
#define __pa_PID_H_

#define Max_iSum 1000
class pa_PID
{
public:
    pa_PID();
    pa_PID(float kp1,float ki1,float kd1);
    float calcPid(float err);
    void setPid(float kp,float ki,float kd);
    float kp;
    float ki;
    float kd;
    float iSum=0;//积分项
private:
    
    float lastErr=0;
};

#endif