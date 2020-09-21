#include "Kalman.h"
#include "math.h"
#include "stdio.h"
#define NDEBUG    //调试时禁用，使用中启用
#include <assert.h>


#define IS_EQUAL(A,B) (A==B)

void KalmanFilterInit(KalmanFilterType* KalmanMatData,unsigned int X_num,unsigned int Z_num,unsigned int U_num)
{
    KalmanMatData->Kalman_X_num=X_num;
    KalmanMatData->Kalman_Z_num=Z_num;
    if(U_num!=0)
    {
        assert(IS_EQUAL(X_num,U_num));
        KalmanMatData->Kalman_U_num=U_num;
        CreatMatrix(&(KalmanMatData->Kalman_B),KalmanMatData->Kalman_U_num,KalmanMatData->Kalman_U_num);
        CreatVector(&(KalmanMatData->Kalman_U),KalmanMatData->Kalman_U_num);
    }
    CreatMatrix(&(KalmanMatData->Kalman_A),KalmanMatData->Kalman_X_num,KalmanMatData->Kalman_X_num);
    CreatMatrix(&(KalmanMatData->Kalman_C),KalmanMatData->Kalman_Z_num,KalmanMatData->Kalman_X_num);
    CreatMatEye(&(KalmanMatData->Kalman_P0),KalmanMatData->Kalman_X_num);
    CreatMatrix(&(KalmanMatData->Kalman_P1),KalmanMatData->Kalman_X_num,KalmanMatData->Kalman_X_num);
    CreatMatrix(&(KalmanMatData->Kalman_R),KalmanMatData->Kalman_X_num,KalmanMatData->Kalman_X_num);
    CreatMatrix(&(KalmanMatData->Kalman_Q),KalmanMatData->Kalman_Z_num,KalmanMatData->Kalman_Z_num);
    CreatVector(&(KalmanMatData->Kalman_X),KalmanMatData->Kalman_X_num);
    CreatVector(&(KalmanMatData->Kalman_Z),KalmanMatData->Kalman_Z_num);
}


void KalmanProcess(KalmanFilterType* KalmanMatData)
{
    VectorType Kalman_X_exp;
    CreatVector(&Kalman_X_exp,KalmanMatData->Kalman_X_num);

    //状态预测
    if(KalmanMatData->Kalman_U_num==0)
    {
        MatMultiVector(&(KalmanMatData->Kalman_A),&(KalmanMatData->Kalman_X),&Kalman_X_exp); 
    }
    else
    {
        VectorType temp1,temp2;
        CreatVector(&temp1,KalmanMatData->Kalman_X_num);
        CreatVector(&temp2,KalmanMatData->Kalman_U_num);
        MatMultiVector(&(KalmanMatData->Kalman_A),&(KalmanMatData->Kalman_X),&temp1); 
        MatMultiVector(&(KalmanMatData->Kalman_B),&(KalmanMatData->Kalman_U),&temp2);
        VectorPlus(&temp1,&temp2,&Kalman_X_exp); 
        FreeVector(&temp1); 
        FreeVector(&temp2); 
    }

    //协方差预测
    
    
}