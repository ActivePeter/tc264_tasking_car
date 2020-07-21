#include "pa_CrossCalc.h"

float getMotorRotationValueByErr(char err){

    char dir = err>0? 1:-1;
    err=err>0?err:-err;

    if(err>4){
        return 800*dir;
    }
    char result=0;
    switch (err)
    {
    case 0:
        return 0*dir;
        break;
    case 1:
        return 300*dir;
        break;
    case 2:
        return 500*dir;
        break;
    case 3:
        return 675*dir;
        break;
    case 4:
        return 800*dir;
        break;
    }
}