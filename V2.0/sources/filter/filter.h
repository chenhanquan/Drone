#ifndef FILTER_H
#define FILTER_H
#include "stm32f4xx.h"

float Low_Filter(float value);
void SortAver_Filter(float value,float *filter,uint8_t N);
void SortAver_Filter1(float value,float *filter,uint8_t n);
void SortAver_FilterXYZ(float *acc,float *Acc_filt,uint8_t N);
void Aver_FilterXYZ6(float *acc,float *gry,float *Acc_filt,float *Gry_filt,uint8_t N);
void Aver_FilterXYZ(float *acc,float *Acc_filt,uint8_t N);
void Aver_Filter(float data,float *filt_data,uint8_t n);
void Aver_Filter1(float data,float *filt_data,uint8_t n);
void presssureFilter(float* in, float* out);

void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq);
float LPF2pApply_1(float sample);


#endif
