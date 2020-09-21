#ifndef _HQMATH_H_
#define _HQMATH_H_

//矩阵操作
void MatCopy(float *to,float *from,unsigned int from_rows,unsigned int from_cols);                //复制矩阵
void MatSwapRow(float *m,unsigned int rows,unsigned int cols, unsigned int i, unsigned int j);    //矩阵交换行
void MatSwapCol(float *m,unsigned int rows,unsigned int cols, unsigned int i, unsigned int j);    //矩阵交换列

void MatTranspose(float* Mat,int rows,int cols,float* Mat_Tran);                                                      //矩阵转置
void GetMatEye(int n,float*eye);                                                                                      //生成单位阵
void MatPlus(float*Mat1,int Mat1_rows,int Mat1_cols,float*Mat2,int Mat2_rows,int Mat2_cols,float* Mat_Add);           //矩阵加法
void MatMinus(float*Mat1,int Mat1_rows,int Mat1_cols,float*Mat2,int Mat2_rows,int Mat2_cols,float* Mat_Minus);        //矩阵减法
void MatMulti(float*Mat1,int Mat1_rows,int Mat1_cols,float*Mat2,int Mat2_rows,int Mat2_cols,float* Mat_Multi);        //矩阵乘法
void MatMultiVector(float*Mat,int rows,int cols,float*Vector,int Vector_num,float* Mat_Multi);                        //矩阵乘向量
void KMat(float*Mat,int rows,int cols,const float K,float* KMat);                                                     //矩阵乘系数
float GetMatDet(float*Mat,int n);                                                                                     //矩阵行列式

#define Guass    //使用高斯-约旦法计算逆矩阵
#ifdef Guass

#define MatInvGuass MatInv                    //重定义函数名，防止程序混乱
#define GetAdjugateMatGuass GetAdjugateMat    //重定义函数名，防止程序混乱

int MatInvGuass(float *a,unsigned int n,float *inv);                                                                  //矩阵的逆
void GetAdjugateMatGuass(float*Mat,unsigned int n,float*AdjMat);                                                      //伴随矩阵

#else
//简单方法，但是当矩阵太大时内存和计算量会剧增
void GetAdjugateMat(float*Mat,unsigned int n,float* AdjMat);                                                          //伴随矩阵
void MatInv(float*Mat,unsigned int n,float* InvMat);                                                                  //矩阵的逆

#endif
//void MatShow(float*Mat,int rows,int cols);                                                              //展示矩阵，一般在调试用

#endif
