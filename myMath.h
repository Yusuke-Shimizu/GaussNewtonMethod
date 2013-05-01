#ifndef MY_MATH_H
#define MY_MATH_H

#include  <math.h>
#include <malloc.h>
#include <stdlib.h>
#include <windows.h>
#include <mmsystem.h>
#pragma comment( lib, "winmm.lib" )

const double myPi = 3.14159265358979323846;

/*-------------------�x�N�g���v�Z------------------*/
void	Vec3Copy(float out[3], const float v[3]);						//	�R�s�[						
void	Vec3Add(float out[3], const float v1[3], const float v2[3]);	//	�a
void	Vec3Sub(float out[3], const float v1[3], const float v2[3]);	//	��
void	Vec3Scale(float out[3], float k, const float v[3]);				//	n�{
float	Vec3Dot(const float v1[3], const float v2[3]);					//	����
void	Vec3Cross(float out[3], const float v1[3], const float v2[3]);	//	�O��
float	Vec3Length(const float v[3]);									//	����
void	Vec3Normalize(float out[3], const float v[3]);					//	���K��
/*----------�N�H�[�^�j�I��-------*/
// �Q�l�Fhttp://marina.sys.wakayama-u.ac.jp/~tokoi/?date=20040321
void QuatMul(double r[], const double p[], const double q[]);	// �N�H�[�^�j�I���̐�
void QuatToRotateMatrix(double r[], double q[]);				// �N�H�[�^�j�I�������]�̕ϊ��s��

/*---------------------�s��v�Z--------------------*/
void MakeTranslateMatrix(float m[16], float x, float y, float z);			// ���s�ړ��s�񐶐�
void TransposeMatrix(float out[16], const float m[16]);						// �]�u�s��쐬
void MultMatrix(float out[16], const float m1[16], const float m2[16]);		// �s��̊|���Z
void TransformVec16(float out[4], const float mat[16], const float vec[4]);	// �s��ƃx�N�g���̊|���Z
void TransformVec3x4(double out[3], const double mat[3][4], const double vec[3]);
void MakeInverseMatrix(const double* inMat, double* outMat,const int n );	// �t�s�񐶐�

/*---------------------������----------------------*/
// �ʏ��3�_����@���x�N�g�������߂� v0->v1->v2 ����肪�\
void GetNormal(float normal[3], const float v0[3], const float v1[3], const float v2[3]);
// �O�p�`�̖ʐς����߂�
double GetTriangleArea(const float v0[3], const float v1[3], const float v2[3]);
// ����
double myRand( const double* min, const double* max);
// FPS�v��
void CountFPS( DWORD* in_fps );
#endif