#ifndef MY_MATH_H
#define MY_MATH_H

#include  <math.h>
#include <malloc.h>
#include <stdlib.h>
#include <windows.h>
#include <mmsystem.h>
#pragma comment( lib, "winmm.lib" )

const double myPi = 3.14159265358979323846;

/*-------------------ベクトル計算------------------*/
void	Vec3Copy(float out[3], const float v[3]);						//	コピー						
void	Vec3Add(float out[3], const float v1[3], const float v2[3]);	//	和
void	Vec3Sub(float out[3], const float v1[3], const float v2[3]);	//	差
void	Vec3Scale(float out[3], float k, const float v[3]);				//	n倍
float	Vec3Dot(const float v1[3], const float v2[3]);					//	内積
void	Vec3Cross(float out[3], const float v1[3], const float v2[3]);	//	外積
float	Vec3Length(const float v[3]);									//	長さ
void	Vec3Normalize(float out[3], const float v[3]);					//	正規化
/*----------クォータニオン-------*/
// 参考：http://marina.sys.wakayama-u.ac.jp/~tokoi/?date=20040321
void QuatMul(double r[], const double p[], const double q[]);	// クォータニオンの積
void QuatToRotateMatrix(double r[], double q[]);				// クォータニオンから回転の変換行列

/*---------------------行列計算--------------------*/
void MakeTranslateMatrix(float m[16], float x, float y, float z);			// 平行移動行列生成
void TransposeMatrix(float out[16], const float m[16]);						// 転置行列作成
void MultMatrix(float out[16], const float m1[16], const float m2[16]);		// 行列の掛け算
void TransformVec16(float out[4], const float mat[16], const float vec[4]);	// 行列とベクトルの掛け算
void TransformVec3x4(double out[3], const double mat[3][4], const double vec[3]);
void MakeInverseMatrix(const double* inMat, double* outMat,const int n );	// 逆行列生成

/*---------------------未分類----------------------*/
// 面上の3点から法線ベクトルを求める v0->v1->v2 左回りが表
void GetNormal(float normal[3], const float v0[3], const float v1[3], const float v2[3]);
// 三角形の面積を求める
double GetTriangleArea(const float v0[3], const float v1[3], const float v2[3]);
// 乱数
double myRand( const double* min, const double* max);
// FPS計測
void CountFPS( DWORD* in_fps );
#endif