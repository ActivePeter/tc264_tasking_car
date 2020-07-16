#include "pa_MecanumModel.h"

void pa_MecanumModel::initModel()
{
    kinematicsModel.velocity.x = 0;
    kinematicsModel.velocity.y = 0;
    kinematicsModel.angularVelocity = 0;

    modelH = 0;
    modelW = 0;
}

pa_MecanumModel::pa_MecanumModel()
{
    initModel();
}

void pa_MecanumModel::updateKinematics(float vx, float vy, float av)
{
    this->kinematicsModel.velocity.x = vx;
    this->kinematicsModel.velocity.y = vy;
    this->kinematicsModel.angularVelocity = av;
}

pa_KinematicsMotorVelocities pa_MecanumModel::calcMotorsStates()
{
    this->motorVelocities.velocityM1 = kinematicsModel.velocity.y - kinematicsModel.velocity.x + kinematicsModel.angularVelocity * (modelW + modelH);
    this->motorVelocities.velocityM2 = kinematicsModel.velocity.y + kinematicsModel.velocity.x - kinematicsModel.angularVelocity * (modelW + modelH);
    this->motorVelocities.velocityM3 = kinematicsModel.velocity.y - kinematicsModel.velocity.x - kinematicsModel.angularVelocity * (modelW + modelH);
    this->motorVelocities.velocityM4 = kinematicsModel.velocity.y + kinematicsModel.velocity.x + kinematicsModel.angularVelocity * (modelW + modelH);
}
