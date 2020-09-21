#include "stdio.h"
#include "HQ_math.h"
#include "math.h"

int main()
{
	MatrixType *Mat1;
	int j =1;
	CreatMatrix(Mat1,3,3);
	Mat1->Matrix[0]=12.0;
	for(unsigned int i=0;i<9;i++)
	{
		Mat1->Matrix[i]=i;
	}

	MatShow(Mat1);
	FreeMatrix(Mat1);
	printf("%d",j);
	return 0;
}