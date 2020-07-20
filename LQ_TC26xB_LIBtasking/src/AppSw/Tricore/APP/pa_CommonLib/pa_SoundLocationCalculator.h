#ifndef __pa_SoundLocationCalculator_H_
#define __pa_SoundLocationCalculator_H_

class pa_SoundLocationCalculator
{
public:
    static char crossCorrelation_noFFT(unsigned short adc_arr1[],unsigned short adc_arr2[],short adc_arrlen);
private:
    
};

#endif