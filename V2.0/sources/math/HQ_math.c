#include <stdio.h>
#include "stdlib.h"
#include "HQ_math.h"
#include "math.h"
#define NDEBUG    //调试时禁用，使用中启用
#include <assert.h>


#define IS_EQUAL(A,B) (A==B)
#define IS_NULL(A) (A==NULL)

#ifdef NODEBUG
#define LM_ASSERT(x)
#else
#define LM_ASSERT(x)	do{ if(!(x)) while(1); }while(0)
#endif

#define MATRIX(M,col, r, c) (*(M + (r)*col + (c)))    //交换单个元素


/*
	brief:矩阵转置
	para:
		Mat:矩阵
		rows:行数
		cols:列数
*/
void MatTranspose(float* Mat,int rows,int cols,float* Mat_Tran)
{
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			Mat_Tran[j*rows+i]=Mat[i*cols+j];
		}
	}
}

/*
	brief:生成单位阵
	para:
		n:阶数
*/
void GetMatEye(int n,float*eye)
{
	  for(int i=0;i<n;i++)
	  {
		for(int j=0;j<n;j++)
        {
            if(i==j){eye[i*n+j]=1.0;}else{eye[i*n+j]=0.0;}
        }	
		}
}

/*
	brief:矩阵相加 Mat1+Mat2
	para:
		Mat1、Mat2:矩阵
		rows:行数
		cols:列数
*/
void MatPlus(float*Mat1,int Mat1_rows,int Mat1_cols,float*Mat2,int Mat2_rows,int Mat2_cols,float* Mat_Add)
{
	
	assert(IS_EQUAL(Mat1_rows,Mat2_rows));
	assert(IS_EQUAL(Mat1_cols,Mat2_cols));
	for(int i=0;i<Mat1_rows;i++)
	{
		for(int j=0;j<Mat1_cols;j++)
		{
			Mat_Add[i*Mat1_cols+j]=Mat1[i*Mat1_cols+j]+Mat2[i*Mat1_cols+j];
		}
	}
}

/*
	brief:矩阵相减 Mat1-Mat2
	para:
		Mat1、Mat2:矩阵
		rows:行数
		cols:列数
*/
void MatMinus(float*Mat1,int Mat1_rows,int Mat1_cols,float*Mat2,int Mat2_rows,int Mat2_cols,float* Mat_Minus)
{
	
	assert(IS_EQUAL(Mat1_rows,Mat2_rows));
	assert(IS_EQUAL(Mat1_cols,Mat2_cols));
	for(int i=0;i<Mat1_rows;i++)
	{
		for(int j=0;j<Mat1_cols;j++)
		{
			Mat_Minus[i*Mat1_cols+j]=Mat1[i*Mat1_cols+j]-Mat2[i*Mat1_cols+j];
		}
	}
}

/*
	brief:矩阵乘法 Mat1*Mat2
	para:
		Mat1、Mat2:矩阵
		rows:行数
		cols:列数
*/
void MatMulti(float*Mat1,int Mat1_rows,int Mat1_cols,float*Mat2,int Mat2_rows,int Mat2_cols,float* Mat_Multi)
{
	
	assert(IS_EQUAL(Mat1_rows,Mat2_cols));
	assert(IS_EQUAL(Mat1_cols,Mat2_rows));
	for(int i=0;i<Mat1_rows;i++)
	{
		for(int j=0;j<Mat2_cols;j++)
		{
			Mat_Multi[i*Mat1_rows+j]=0.0;
			for(int k=0;k<Mat1_cols;k++){Mat_Multi[i*Mat2_cols+j]+=(Mat1[i*Mat1_cols+k]*Mat2[k*Mat2_cols+j]);}
		}
	}
}

/*
	brief:矩阵乘向量 Mat*Vector
	para:
		Mat:矩阵
		rows:行数
		cols:列数
		Vector:向量
		Vector_num:向量元素个数
*/
void MatMultiVector(float*Mat,int rows,int cols,float*Vector,int Vector_num,float* Mat_Multi)
{
	
	assert(IS_EQUAL(cols,Vector_num));
	for(int i=0;i<rows;i++)
	{
		Mat_Multi[i]=0.0;
		for(int j=0;j<cols;j++)
		{
			Mat_Multi[i]+=Mat[i*cols+j]*Vector[j];
		}
	}
}

/*
	brief:矩阵乘系数 Mat*K
	para:
		Mat:矩阵
		rows:行数
		cols:列数
		K:系数
*/
void KMat(float*Mat,int rows,int cols,const float K,float* KMat)
{
	KMat=(float*)malloc(sizeof(float)*rows*cols);
	assert(IS_NULL(KMat));
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			KMat[i*cols+j]=K*Mat[i*cols+j];				
		}
	}
}

/*
	brief:矩阵行列式
	para:
		Mat:矩阵
		n:矩阵阶数
*/
float GetMatDet(float*Mat,int n)
{
    float num=0;
    if (n == 2)
	{
        num = Mat[0] * Mat[3] - Mat[1] * Mat[2];
    }
    else
	{
        for(int i=0;i<n;i++)
        {
            float* Mat_A=NULL;
            float flag=0;
            if(Mat[i]==0.0){continue;}
            Mat_A=(float*)malloc(sizeof(float)*(n-1)*(n-1));
            assert(IS_NULL(Mat_A));
            for(int j=0;j<(n-1);j++)
            {
                for(int k=0;k<(n-1);k++)
                {
                    if(k<i)
                    {
                        Mat_A[j*(n-1)+k]=Mat[(j+1)*n+k];
                    }
                    else
                    {
                        Mat_A[j*(n-1)+k]=Mat[(j+1)*n+k+1];
                    }
                }
            }
            if(i%2==0){flag=1;}else{flag=-1;}
            num+=Mat[i]*flag*GetMatDet(Mat_A,(n-1));
            free(Mat_A);
        }

    }
    return num;
}

#ifdef Guass

/**
 * @brief 求逆矩阵
 */
int MatInvGuass(float *a,unsigned int n,float *inv)
{
	int i, j, k;

	unsigned int is[n];
	unsigned int js[n];

	float max;

	MatCopy(inv,a,n,n);

	for(k=0; k<n; k++)
	{
		//step 1, 全选主元
		max = 0;
		is[k] = k;
		js[k] = k;

		for(i=k; i<n; i++)
		{
			for(j=k; j<n; j++)
			{
				if(max < fabs(MATRIX(inv,n, i, j)))
				{
					max = fabs(MATRIX(inv,n, i, j));
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if(max < 0.0001)
		{	//! 无逆矩阵
			return -1;
		}

		//交换
		if(is[k] != k)
		{
			MatSwapRow(inv,n,n, k, is[k]);
		}
		if(js[k] != k)
		{
			MatSwapCol(inv,n,n, k, js[k]);
		}

		MATRIX(inv,n, k, k) = 1 / MATRIX(inv,n, k, k);

		for(j=0; j<n; j++)
		{
			if(j != k)
				MATRIX(inv,n, k, j) *= MATRIX(inv,n, k, k);
		}
		for(i=0; i<n; i++)
		{
			if(i != k)
			{
				for(j=0; j<n; j++)
				{
					if(j != k)
						MATRIX(inv,n, i, j) -= MATRIX(inv,n, i, k) * MATRIX(inv,n, k, j);
				}
			}
		}
		for(i=0; i<n; i++)
		{
			if(i != k)
				MATRIX(inv,n, i, k) *= -MATRIX(inv,n, k, k);
		}

	}

	//恢复
	//本来 row <-> is[k], column <-> js[k]
	//恢复时：row <-> js[k], column <-> is[k]
	for(k=n-1; k>=0; k--)
	{
		if(js[k] != k)
		{
			MatSwapRow(inv,n,n, k, js[k]);
		}
		if(is[k] != k)
		{
			MatSwapCol(inv,n,n, k, is[k]);
		}
	}
	return 1;
}

void GetAdjugateMatGuass(float*Mat,unsigned int n,float*AdjMat)
{
	float temp[n];
	float det=0;
	MatInvGuass(Mat,n,temp);
	det=GetMatDet(Mat,n);
	KMat(temp,n,n,det,AdjMat);
}

#else

/*
	brief:伴随矩阵
	para:
		Mat:矩阵
		n:矩阵阶数
*/
void GetAdjugateMat(float*Mat,unsigned int n,float* AdjMat)
{
	float* AdjMat_Temp=NULL; 
	AdjMat_Temp=(float*)malloc(sizeof(float)*n*n);
	assert(IS_NULL(AdjMat_Temp));
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<n;j++)
		{
			float*MatTemp=NULL;
            float flag=0;
            if((i+j)%2==0){flag=1;}else{flag=-1;}
			MatTemp=(float*)malloc(sizeof(float)*(n-1)*(n-1));
			assert(IS_NULL(MatTemp));
			for(int w=0;w<(n-1);w++)
			{
				for(int v=0;v<(n-1);v++)
				{
					if(w<i)
					{
						if(v<j){MatTemp[w*(n-1)+v]=Mat[w*n+v];}else{MatTemp[w*(n-1)+v]=Mat[w*n+v+1];}
					}
					else
					{
						if(v<j){MatTemp[w*(n-1)+v]=Mat[(w+1)*n+v];}else{MatTemp[w*(n-1)+v]=Mat[(w+1)*n+v+1];}
					}								
				}
			}
			AdjMat_Temp[i*n+j]=flag*GetMatDet(MatTemp,n-1);
			free(MatTemp);
		}
	}
	MatTranspose(AdjMat_Temp,n,n,AdjMat);
	free(AdjMat_Temp);
}

/*
	brief:矩阵求逆
	para:
		Mat:矩阵
		n:矩阵阶数
*/
void MatInv(float*Mat,unsigned int n,float* InvMat)
{
	float* AdjMat=NULL;
	float det;
	AdjMat=(float*)malloc(sizeof(float)*n*n);
	GetAdjugateMat(Mat,n,AdjMat);
	det=GetMatDet(Mat,n);
	KMat(AdjMat,n,n,(1/det),InvMat);
	free(AdjMat);
}

#endif

/**
 * @brief 交换行
 * i:原行
 * j:要换到的行
 */
void MatSwapRow(float *m,unsigned int rows,unsigned int cols, unsigned int i, unsigned int j)
{
	unsigned int k;
	float tmp;

	LM_ASSERT(i < rows);
	LM_ASSERT(j < rows);

	for(k=0; k < cols; k++)
	{
		tmp = MATRIX(m,cols, i, k);
		MATRIX(m,cols, i, k) = MATRIX(m,cols, j, k);
		MATRIX(m,cols, j, k) = tmp;
	}
}

/**
 * @brief 交换列
 * i:原列
 * j:要换到的列
 */
void MatSwapCol(float *m,unsigned int rows,unsigned int cols, unsigned int i, unsigned int j)
{
	unsigned int k;
	float tmp;

	LM_ASSERT(i <cols);
	LM_ASSERT(j < cols);

	for(k=0; k<rows; k++)
	{
		tmp = MATRIX(m,cols, k, i);
		MATRIX(m,cols, k, i) = MATRIX(m,cols, k, j);
		MATRIX(m,cols, k, j) = tmp;
	}
}

/**
 * @brief 复制矩阵
 */
void MatCopy(float *to,float *from,unsigned int from_rows,unsigned int from_cols)
{
	unsigned int i, j;

	for(i=0; i<from_rows; i++)
		for(j=0; j<from_cols; j++)
			MATRIX(to,from_cols, i, j) = MATRIX(from,from_cols, i, j);
}



///*
//	brief:矩阵显示（一般在调试中使用）
//	para:
//		Mat:矩阵
//*/
//void MatShow(float*Mat,int rows,int cols)
//{
//    printf("\n-------------------------------\n");
//    for(int i=0;i<rows;i++)
//    {
//        for(int j=0;j<cols;j++)
//        {
//            printf("%.3f\t",Mat[i*cols+j]);
//        }
//        printf("\n");
//    }
//    printf("\n-------------------------------\n");
//}
