#include "pa_MotorManager.h"

void pa_initMotorPwm()
{
    //motor1
    TOM_PWM_InitConfig(PWM5, 0, 15000); //初始化P33_10 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
    TOM_PWM_InitConfig(PWM6, 0, 15000); //初始化P33_13 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100

    //motor2
    TOM_PWM_InitConfig(PWM3, 0, 15000); //初始化P33_10 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
    TOM_PWM_InitConfig(PWM4, 0, 15000); //初始化P33_6 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100

    //motor3
    TOM_PWM_InitConfig(PWM1, 0, 15000); //初始化P33_13 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
    TOM_PWM_InitConfig(PWM2, 0, 15000); //初始化P33_7 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100

    //motor4
    TOM_PWM_InitConfig(PWM8, 0, 15000); //初始化P33_6 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
    TOM_PWM_InitConfig(PWM7, 0, 15000); //初始化P33_7 作为PWM输出口 频率100Hz 占空比 百分之(5000/TOM_PWM_MAX)*100
}

void pa_updateMotorPwm(char index, float speed)
{

    switch (index)
    {
    case 1:
        speed*=1.19444f;
        break;
    case 2:
        speed*=1.075f;
        /* code */
        break;
    case 3:
        
        /* code */
        break;
    case 4:
        
        /* code */
        break;
    default:
        break;
    }
    int pwmDuty = (int)speed;
    char dir=speed>0;
    if(!dir)pwmDuty=-pwmDuty;
    pwmDuty+=100;
    if(pwmDuty>5000){
        pwmDuty=5000;
    }
    dir=!dir;
    switch (index)
    {
    case 1:
        if (dir)
        {
            TOM_PWM_SetDuty(PWM5,pwmDuty,15000);
            TOM_PWM_SetDuty(PWM6,0,15000);
        }
        else
        {
            TOM_PWM_SetDuty(PWM6,pwmDuty,15000);
            TOM_PWM_SetDuty(PWM5,0,15000);
        }
        /* code */
        break;
    case 2:
        if (dir)
        {
            TOM_PWM_SetDuty(PWM3,pwmDuty,15000);
            TOM_PWM_SetDuty(PWM4,0,15000);
        }
        else
        {
            TOM_PWM_SetDuty(PWM4,pwmDuty,15000);
            TOM_PWM_SetDuty(PWM3,0,15000);
        }
        /* code */
        break;
    case 3:
        if (dir)
        {
            TOM_PWM_SetDuty(PWM1,pwmDuty,15000);
            TOM_PWM_SetDuty(PWM2,0,15000);
        }
        else
        {
            TOM_PWM_SetDuty(PWM2,pwmDuty,15000);
            TOM_PWM_SetDuty(PWM1,0,15000);
        }
        /* code */
        break;
    case 4:
        if (dir)
        {
            TOM_PWM_SetDuty(PWM8,pwmDuty,15000);
            TOM_PWM_SetDuty(PWM7,0,15000);
        }
        else
        {
            TOM_PWM_SetDuty(PWM7,pwmDuty,15000);
            TOM_PWM_SetDuty(PWM8,0,15000);
        }
        /* code */
        break;
    default:
        break;
    }
}
