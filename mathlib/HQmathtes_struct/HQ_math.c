#include <stdio.h>
#include "stdlib.h"
#include "HQ_math.h"
#include "math.h"
//#define NDEBUG    //调试时禁用，使用中启用
#include <assert.h>


#define IS_EQUAL(A,B) (A==B)
#define IS_NULL(A) (A==NULL)

#ifdef NODEBUG
#define LM_ASSERT(x)
#else
#define LM_ASSERT(x)	do{ if(!(x)) while(1); }while(0)
#endif

#define MATRIX(M, r, c) (*((M->Matrix) + (r)*(M->cols) + (c)))    //交换单个元素

void CreatMatrix(MatrixType *Mat,int rows,int cols)
{
	Mat->rows=rows;
	Mat->cols=cols;
	Mat->Matrix=(float*)malloc(sizeof(float)*rows*cols);
	assert(IS_NULL(Mat->Matrix));
	for(int i=0;i<(rows*cols);i++)
	{
		Mat->Matrix[i]=8.0;
	}
}

void CreatVector(VectorType *Vec,int num)
{
	Vec->num=num;
	Vec->Vector=(float*)malloc(sizeof(float)*num);
	for(int i;i<num;i++)
	{
		Vec->Vector[i]=0;
	}
}

void FreeMatrix(MatrixType *Mat)
{
	Mat->rows=0;
	Mat->cols=0;
	free(Mat->Matrix);
}

void FreeVector(VectorType *Vec)
{
	Vec->num=0;
	free(Vec->Vector);
}
/*
	brief:矩阵转置
	para:
		Mat:矩阵
		rows:行数
		cols:列数
*/
void MatTranspose(MatrixType *Mat)
{
	MatrixType *Mat_temp=NULL;
	MatCopy(Mat,Mat_temp);
	for(int i=0;i<Mat->rows;i++)
	{
		for(int j=0;j<Mat->cols;j++)
		{
			Mat->Matrix[j*(Mat->rows)+i]=Mat_temp->Matrix[i*(Mat->cols)+j];
		}
	}
	FreeMatrix(Mat_temp);
}

/*
	brief:生成单位阵
	para:
		n:阶数
*/
void CreatMatEye(MatrixType*EyeMat,int n)
{
	CreatMatrix(EyeMat,n,n);
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<n;j++)
        {
            if(i==j){EyeMat->Matrix[i*n+j]=1.0;}else{EyeMat->Matrix[i*n+j]=0.0;}
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
void MatPlus(MatrixType *Mat1,MatrixType *Mat2,MatrixType *Mat_Add)
{
	assert(IS_EQUAL(Mat1->rows,Mat2->rows));
	assert(IS_EQUAL(Mat1->cols,Mat2->cols));
	assert(IS_EQUAL(Mat1->rows,Mat_Add->rows));
	assert(IS_EQUAL(Mat1->cols,Mat_Add->cols));
	for(int i=0;i<(Mat1->rows);i++)
	{
		for(int j=0;j<(Mat1->cols);j++)
		{
			Mat_Add->Matrix[i*Mat1->cols+j]=Mat1->Matrix[i*Mat1->cols+j]+Mat2->Matrix[i*Mat1->cols+j];
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
void MatMinus(MatrixType *Mat1,MatrixType *Mat2,MatrixType *Mat_Minus)
{
	assert(IS_EQUAL(Mat1->rows,Mat2->rows));
	assert(IS_EQUAL(Mat1->cols,Mat2->cols));
	assert(IS_EQUAL(Mat1->rows,Mat_Minus->rows));
	assert(IS_EQUAL(Mat1->cols,Mat_Minus->cols));
	for(int i=0;i<(Mat1->rows);i++)
	{
		for(int j=0;j<(Mat1->cols);j++)
		{
			Mat_Minus->Matrix[i*Mat1->cols+j]=Mat1->Matrix[i*Mat1->cols+j]-Mat2->Matrix[i*Mat1->cols+j];
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
void MatMulti(MatrixType *Mat1,MatrixType *Mat2,MatrixType *Mat_Multi)
{
	
	assert(IS_EQUAL(Mat1->rows,Mat2->cols));
	assert(IS_EQUAL(Mat1->cols,Mat2->rows));
	assert(IS_EQUAL(Mat1->rows,Mat_Multi->rows));
	assert(IS_EQUAL(Mat2->cols,Mat_Multi->cols));
	for(int i=0;i<Mat1->rows;i++)
	{
		for(int j=0;j<Mat2->cols;j++)
		{
			Mat_Multi->Matrix[i*Mat1->rows+j]=0.0;
			for(int k=0;k<Mat1->cols;k++){Mat_Multi->Matrix[i*Mat2->cols+j]+=(Mat1->Matrix[i*Mat1->cols+k]*Mat2->Matrix[k*Mat2->cols+j]);}
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
void MatMultiVector(MatrixType *Mat,VectorType *Vec,VectorType *Vec_out)
{
	
	assert(IS_EQUAL(Mat->cols,Vec->num));
	assert(IS_EQUAL(Vec_out->num,Vec->num));
	for(int i=0;i<Mat->rows;i++)
	{
		Vec_out->Vector[i]=0.0;
		for(int j=0;j<Mat->cols;j++)
		{
			Vec_out->Vector[i]+=Mat->Matrix[i*Mat->cols+j]*Vec->Vector[j];
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
void KMat(MatrixType *Mat,const float K)
{
	MatrixType *Mat_temp=NULL;
	MatCopy(Mat,Mat_temp); 
	for(int i=0;i<Mat->rows;i++)
	{
		for(int j=0;j<Mat->cols;j++)
		{
			Mat->Matrix[i*Mat->cols+j]=K*Mat_temp->Matrix[i*Mat->cols+j];				
		}
	}
	FreeMatrix(Mat_temp);
}

/*
	brief:矩阵行列式
	para:
		Mat:矩阵
		n:矩阵阶数
*/
float GetMatDet(MatrixType *Mat)
{
	assert(IS_EQUAL(Mat->cols,Mat->rows));
    float num=0;
    if (Mat->rows == 2)
	{
        num = Mat->Matrix[0] * Mat->Matrix[3] - Mat->Matrix[1] * Mat->Matrix[2];
    }
    else
	{
        for(int i=0;i<Mat->rows;i++)
        {
            MatrixType* Mat_A=NULL;
            float flag=0;
            if(Mat->Matrix[i]==0.0){continue;}
            CreatMatrix(Mat_A,(Mat->rows-1),(Mat->cols-1));
            for(int j=0;j<(Mat->rows-1);j++)
            {
                for(int k=0;k<(Mat->rows-1);k++)
                {
                    if(k<i)
                    {
                        Mat_A->Matrix[j*(Mat->rows-1)+k] = Mat->Matrix[(j+1)*Mat->rows+k];
                    }
                    else
                    {
                        Mat_A->Matrix[j*(Mat->rows-1)+k]=Mat->Matrix[(j+1)*Mat->rows+k+1];
                    }
                }
            }
            if(i%2==0){flag=1;}else{flag=-1;}
            num+=Mat->Matrix[i]*flag*GetMatDet(Mat_A);
            FreeMatrix(Mat_A);
        }

    }
    return num;
}


/**
 * @brief 求逆矩阵
 */
int MatInv(MatrixType *Mat,MatrixType *Mat_inv)
{
	int i, j, k;

	assert(IS_EQUAL(Mat->cols,Mat->rows));
	assert(IS_EQUAL(Mat_inv->cols,Mat_inv->rows));
	assert(IS_EQUAL(Mat->cols,Mat_inv->rows));

	unsigned int is[Mat->rows];
	unsigned int js[Mat->rows];

	float max;

	MatCopy(Mat_inv,Mat);

	for(k=0; k<Mat->rows; k++)
	{
		//step 1, 全选主元
		max = 0;
		is[k] = k;
		js[k] = k;

		for(i=k; i<Mat->rows; i++)
		{
			for(j=k; j<Mat->rows; j++)
			{
				if(max < fabs(MATRIX(Mat_inv, i, j)))
				{
					max = fabs(MATRIX(Mat_inv, i, j));
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
			MatSwapRow(Mat_inv, k, is[k]);
		}
		if(js[k] != k)
		{
			MatSwapCol(Mat_inv, k, js[k]);
		}

		MATRIX(Mat_inv, k, k) = 1 / MATRIX(Mat_inv, k, k);

		for(j=0; j<Mat->rows; j++)
		{
			if(j != k)
				MATRIX(Mat_inv, k, j) *= MATRIX(Mat_inv, k, k);
		}
		for(i=0; i<Mat->rows; i++)
		{
			if(i != k)
			{
				for(j=0; j<Mat->rows; j++)
				{
					if(j != k)
						MATRIX(Mat_inv, i, j) -= MATRIX(Mat_inv, i, k) * MATRIX(Mat_inv, k, j);
				}
			}
		}
		for(i=0; i<Mat->rows; i++)
		{
			if(i != k)
				MATRIX(Mat_inv, i, k) *= -MATRIX(Mat_inv, k, k);
		}

	}

	//恢复
	//本来 row <-> is[k], column <-> js[k]
	//恢复时：row <-> js[k], column <-> is[k]
	for(k=Mat->rows-1; k>=0; k--)
	{
		if(js[k] != k)
		{
			MatSwapRow(Mat_inv, k, js[k]);
		}
		if(is[k] != k)
		{
			MatSwapCol(Mat_inv, k, is[k]);
		}
	}
	return 1;
}

void GetAdjugateMat(MatrixType *Mat,MatrixType *Mat_Adj)
{
	assert(IS_EQUAL(Mat->cols,Mat->rows));
	assert(IS_EQUAL(Mat->rows,Mat_Adj->rows));
	assert(IS_EQUAL(Mat->cols,Mat_Adj->cols));
	float det=0;
	MatInv(Mat,Mat_Adj);
	det=GetMatDet(Mat);
	KMat(Mat_Adj,det);
}


/**
 * @brief 交换行
 * i:原行
 * j:要换到的行
 */
void MatSwapRow(MatrixType *Mat, unsigned int i, unsigned int j)
{
	unsigned int k;
	float tmp;

	LM_ASSERT(i < Mat->rows);
	LM_ASSERT(j < Mat->rows);

	for(k=0; k < Mat->cols; k++)
	{
		tmp = MATRIX(Mat, i, k);
		MATRIX(Mat, i, k) = MATRIX(Mat, j, k);
		MATRIX(Mat, j, k) = tmp;
	}
}

/**
 * @brief 交换列
 * i:原列
 * j:要换到的列
 */
void MatSwapCol(MatrixType *Mat, unsigned int i, unsigned int j)
{
	unsigned int k;
	float tmp;

	LM_ASSERT(i <Mat->cols);
	LM_ASSERT(j < Mat->cols);

	for(k=0; k<Mat->rows; k++)
	{
		tmp = MATRIX(Mat, k, i);
		MATRIX(Mat, k, i) = MATRIX(Mat, k, j);
		MATRIX(Mat, k, j) = tmp;
	}
}

/**
 * @brief 复制矩阵
 */
void MatCopy(MatrixType *Mat,MatrixType *Mat_copy)
{
	unsigned int i, j;

	assert(IS_EQUAL(Mat->cols,Mat_copy->cols));
	assert(IS_EQUAL(Mat->rows,Mat_copy->rows));

	for(i=0; i<Mat->rows; i++)
		for(j=0; j<Mat->cols; j++)
			MATRIX(Mat_copy, i, j) = MATRIX(Mat, i, j);
}



/*
	brief:矩阵显示（一般在调试中使用）
	para:
		Mat:矩阵
*/
void MatShow(MatrixType *Mat)
{
   printf("\n-------------------------------\n");
   for(int i=0;i<Mat->rows;i++)
   {
       for(int j=0;j<Mat->cols;j++)
       {
           printf("%.3f\t",Mat[i*Mat->cols+j]);
       }
       printf("\n");
   }
   printf("\n-------------------------------\n");
}


void VectorPlus(VectorType *Vec1,VectorType *Vec2,VectorType *Vec_Add)
{
	assert(IS_EQUAL(Vec1->num,Vec2->num));
	assert(IS_EQUAL(Vec1->num,Vec_Add->num));

	for(unsigned int i=0;i<Vec1->num;i++)
	{
		Vec_Add->Vector[i]=Vec1->Vector[i]+Vec2->Vector[i];
	}
}

void VectorMinus(VectorType *Vec1,VectorType *Vec2,VectorType *Vec_Minus)
{
	assert(IS_EQUAL(Vec1->num,Vec2->num));
	assert(IS_EQUAL(Vec1->num,Vec_Minus->num));

	for(unsigned int i=0;i<Vec1->num;i++)
	{
		Vec_Minus->Vector[i]=Vec1->Vector[i]-Vec2->Vector[i];
	}
}
