#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "HQ_math.h"

typedef struct 
{
    MatrixType Kalman_A;
    MatrixType Kalman_B;
    MatrixType Kalman_C;
    MatrixType Kalman_P0;
    MatrixType Kalman_P1;
    MatrixType Kalman_Q;
    MatrixType Kalman_R;
    VectorType Kalman_X;
    VectorType Kalman_U;
    VectorType Kalman_Z;
    unsigned int Kalman_X_num;
    unsigned int Kalman_Z_num;
    unsigned int Kalman_U_num;
}KalmanFilterType;









#endif