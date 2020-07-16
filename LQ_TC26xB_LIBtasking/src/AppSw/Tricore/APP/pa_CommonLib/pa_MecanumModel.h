#ifndef __pa_MecanumModel_H_
#define __pa_MecanumModel_H_

typedef struct
{
    float x;
    float y;
} pa_Vector;

typedef struct
{
    float angularVelocity;
    pa_Vector velocity;
} pa_KinematicsModel;

typedef struct
{
    float velocityM1;
    float velocityM2;
    float velocityM3;
    float velocityM4;
} pa_KinematicsMotorVelocities;

class pa_MecanumModel
{
public:
    pa_MecanumModel();
    pa_KinematicsModel kinematicsModel;
    void updateKinematics(float vx, float vy, float av);
    pa_KinematicsMotorVelocities calcMotorsStates();

private:
    void initModel();
    short modelW;
    short modelH;
    pa_KinematicsMotorVelocities motorVelocities;
};

#endif