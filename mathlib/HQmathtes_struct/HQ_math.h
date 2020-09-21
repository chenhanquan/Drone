#ifndef _HQMATH_H_
#define _HQMATH_H_

//数据类型
typedef struct
{
    float *Matrix;
    int rows;
    int cols;
}MatrixType;    //矩阵类型

typedef struct
{
    float *Vector;
    int num;
}VectorType;    //向量类型


//矩阵操作
void CreatMatrix(MatrixType*Mat,int rows,int cols);                         //创建矩阵，调用一次必须释放一次
void CreatVector(VectorType*Vec,int num);                                   //创建向量，调用一次必须释放一次
void CreatMatEye(MatrixType*Mat,int n);                                     //生成单位阵，调用一次必须释放一次
void MatCopy(MatrixType *Mat,MatrixType *Mat_copy);                         //复制矩阵，将 Mat 复制到 Mat_copy
void MatSwapRow(MatrixType *Mat, unsigned int i, unsigned int j);           //矩阵交换行
void MatSwapCol(MatrixType *Mat, unsigned int i, unsigned int j);           //矩阵交换列
void FreeMatrix(MatrixType *Mat);                                           //释放矩阵
void FreeVector(VectorType *Vec);                                           //释放向量

void MatTranspose(MatrixType *Mat);                                                     //矩阵转置                                                                   
void MatPlus(MatrixType *Mat1,MatrixType *Mat2,MatrixType *Mat_Add);                    //矩阵加法    Mat_Add=Mat1+Mat2
void MatMinus(MatrixType *Mat1,MatrixType *Mat2,MatrixType *Mat_Minus);                 //矩阵减法    Mat_Minus=Mat1-Mat2
void MatMulti(MatrixType *Mat1,MatrixType *Mat2,MatrixType *Mat_Multi);                 //矩阵乘法    Mat_Multi=Mat1*Mat2
void MatMultiVector(MatrixType *Mat,VectorType *Vec,VectorType *Vec_out);               //矩阵乘向量  Vec_out=Mat*Vec
void KMat(MatrixType *Mat,const float K);                                               //矩阵乘系数  Mat=K*Mat
float GetMatDet(MatrixType *Mat);                                                       //矩阵行列式
int MatInv(MatrixType *Mat,MatrixType *Mat_inv);                                        //矩阵的逆,使用高斯-约旦法
void GetAdjugateMat(MatrixType *Mat,MatrixType *Mat_Adj);                               //伴随矩阵    A*=det*A_
void MatShow(MatrixType *Mat);                                                          //展示矩阵，一般在调试用

void VectorPlus(VectorType *Vec1,VectorType *Vec2,VectorType *Vec_Add);       //向量相加
void VectorMinus(VectorType *Vec1,VectorType *Vec2,VectorType *Vec_Minus);    //向量相减

#endif
