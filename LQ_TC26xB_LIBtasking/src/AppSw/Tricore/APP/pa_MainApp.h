#ifndef __pa_MainApp_H_
#define __pa_MainApp_H_
#define PI 3.1415926535898

//#include "pa_CommonLib/pa_PID.h"

void startMainTask();
void getadc();
char crossCorrelation_noFFT(unsigned short arr1[],unsigned short arr2[]);
void linearInterpolation();
void addValueToArr(unsigned short value1, unsigned short value2,unsigned short value3, unsigned short value4);
void paLinearInterpolation(unsigned short to1,unsigned short to2,unsigned short count);
void initVariable();
void initFuncs();
void checkSignalStable();
void checkBeaconOn();

typedef struct {
    int speedOfM1;
    int speedOfM2;
    int speedOfM3;
    int speedOfM4;
} SpeedOfMotors;



#endif
