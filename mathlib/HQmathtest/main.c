#include "stdio.h"
#include "HQ_math.h"
#include "math.h"

int main()
{
	float Mat1[9]={1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0};
	float Mat_tran[9];
	MatTranspose(Mat1,3,3,Mat_tran);
	MatShow(Mat_tran,3,3);
	return 0;
}