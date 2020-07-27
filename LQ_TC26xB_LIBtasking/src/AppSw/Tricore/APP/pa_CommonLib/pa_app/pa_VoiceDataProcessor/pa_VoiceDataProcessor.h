#ifndef __pa_VoiceDataProcessor_H_
#define __pa_VoiceDataProcessor_H_

#define adc_arrlen 2048

class pa_VoiceDataProcessor
{
public:
    pa_VoiceDataProcessor();
    void checkBeaconOn();
    void addAdcValueToArr(unsigned short value1, unsigned short value2, unsigned short value3, unsigned short value4,unsigned short value5);
    char getErrX();
    char getErrY();
    char isBeaconOn();
    short distance=0;
private:
    unsigned short adc_arr1[adc_arrlen];
    unsigned short adc_arr2[adc_arrlen];
    unsigned short adc_arr3[adc_arrlen];
    unsigned short adc_arr4[adc_arrlen];
    unsigned short adc_arrFm[adc_arrlen];

    unsigned short step = 0;
    short rightFftCount = 0; //符合信标阈值的计数。数量超过一定范围确定为信标响起
    short lastrightFftCount = 0;
    char crossCorrelation_noFFT(unsigned short arr1[], unsigned short arr2[]);
};

#endif