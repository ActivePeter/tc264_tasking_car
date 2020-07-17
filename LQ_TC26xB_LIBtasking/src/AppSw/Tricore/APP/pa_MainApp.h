#ifndef __pa_MainApp_H_
#define __pa_MainApp_H_
#define PI 3.1415926535898

void startMainTask();
void getadc();
void crossCorrelation_noFFT();
void linearInterpolation();
void addValueToArr(unsigned short value1,unsigned short value2);
void paLinearInterpolation(unsigned short to1,unsigned short to2,unsigned short count);
void initVariable();
void initFuncs();
void checkUartData();
typedef struct {
    int speedOfM1;
    int speedOfM2;
    int speedOfM3;
    int speedOfM4;
} SpeedOfMotors;
#endif
