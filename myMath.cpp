#include  "myMath.h"

void Vec3Copy(float out[3], const float v[3])
{
	out[0] = v[0];
	out[1] = v[1];
	out[2] = v[2];
}

void Vec3Add(float out[3], const float v1[3], const float v2[3])
{
	out[0] = v1[0] + v2[0];
	out[1] = v1[1] + v2[1];
	out[2] = v1[2] + v2[2];
}

void Vec3Sub(float out[3], const float v1[3], const float v2[3])
{
	out[0] = v1[0] - v2[0];
	out[1] = v1[1] - v2[1];
	out[2] = v1[2] - v2[2];
}

void Vec3Scale(float out[3], float k, const float v[3])
{
	out[0] = k * v[0];
	out[1] = k * v[1];
	out[2] = k * v[2];
}

float Vec3Dot(const float v1[3], const float v2[3])
{
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

void Vec3Cross(float out[3], const float v1[3], const float v2[3])
{
	out[0] = v1[1]*v2[2] - v1[2]*v2[1];
	out[1] = v1[2]*v2[0] - v1[0]*v2[2];
	out[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

float Vec3Length(const float v[3])
{
	return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void Vec3Normalize(float out[3], const float v[3])
{
	float  len;

	len = Vec3Length(v);

	if (len != 0) {
		out[0] = v[0] / len;
		out[1] = v[1] / len;
		out[2] = v[2] / len;
	}
}

/* ���s�ړ��s����쐬 */
void MakeTranslateMatrix(float m[16], float x, float y, float z)
{
	m[0]  = 1;  m[1]  = 0;  m[2]  = 0;  m[3]  = x;
	m[4]  = 0;  m[5]  = 1;  m[6]  = 0;  m[7]  = y;
	m[8]  = 0;  m[9]  = 0;  m[10] = 1;  m[11] = z;
	m[12] = 0;  m[13] = 0;  m[14] = 0;  m[15] = 1;
}

/* �]�u�s����쐬 */
void TransposeMatrix(float out[16], const float m[16])
{
	float  tmp[16];

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			tmp[i*4 + j] = m[j*4 + i];
		}
	}
	for (int i = 0; i < 16; i++) {
		out[i] = tmp[i];
	}
}

/* �s��̊|���Z */
void MultMatrix(float out[16],
				const float m1[16], const float m2[16])
{
	float tmp[16];
	int i, j;

	for (i=0; i<4; i++) {
		for (j=0; j<4; j++) {
			tmp[i*4+j] = m1[i*4+0] * m2[0*4+j] +
				m1[i*4+1] * m2[1*4+j] +
				m1[i*4+2] * m2[2*4+j] +
				m1[i*4+3] * m2[3*4+j];
		}
	}

	for (i=0; i<16; i++)
		out[i] = tmp[i];
}

/* �s��ƃx�N�g���̊|���Z */
void TransformVec16(float out[4],
			   const float mat[16], const float vec[4])
{
	float tmp[4];
	int i;

	for (i=0; i<4; i++) {
		tmp[i] = mat[i*4+0] * vec[0] +
			mat[i*4+1] * vec[1] +
			mat[i*4+2] * vec[2] +
			mat[i*4+3] * vec[3];
	}

	for (i=0; i<3; i++)
		out[i] = tmp[i] / tmp[3];
	out[3] = 1;
}

void TransformVec3x4(double out[3], const double mat[3][4], const double vec[3])
{
	out[0] = mat[0][0]*vec[0] + mat[0][1]*vec[1] + mat[0][2]*vec[2];
	out[1] = mat[1][0]*vec[0] + mat[1][1]*vec[1] + mat[1][2]*vec[2];
	out[2] = mat[2][0]*vec[0] + mat[2][1]*vec[1] + mat[2][2]*vec[2];
}

/* �ʏ��3�_����@���x�N�g�������߂�
** v0->v1->v2 ����肪�\
*/
void GetNormal(float normal[3], const float v0[3], const float v1[3], 
			   const float v2[3])
{
	float  va[3];
	float  vb[3];
	float  cross[3];

	Vec3Sub(va, v1, v0);
	Vec3Sub(vb, v2, v0);
	Vec3Cross(cross, va, vb);
	Vec3Normalize(normal, cross);
}

/* �O�p�`�̖ʐς����߂� */
double GetTriangleArea(const float v0[3], const float v1[3], const float v2[3])
{
	double  area;
	float  va[3];
	float  vb[3];
	float  cross[3];

	Vec3Sub(va, v1, v0);
	Vec3Sub(vb, v2, v0);
	Vec3Cross(cross, va, vb);
	area = 0.5 * Vec3Length(cross);

	return area;
}

/*
** �N�H�[�^�j�I���̐� r <- p x q
*/
void QuatMul(double r[], const double p[], const double q[])
{
	r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
	r[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
	r[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
	r[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
}

/*
** ��]�̕ϊ��s�� r <- �N�H�[�^�j�I�� q
*/
void QuatToRotateMatrix(double r[], double q[])
{
	double x2 = q[1] * q[1] * 2.0;
	double y2 = q[2] * q[2] * 2.0;
	double z2 = q[3] * q[3] * 2.0;
	double xy = q[1] * q[2] * 2.0;
	double yz = q[2] * q[3] * 2.0;
	double zx = q[3] * q[1] * 2.0;
	double xw = q[1] * q[0] * 2.0;
	double yw = q[2] * q[0] * 2.0;
	double zw = q[3] * q[0] * 2.0;

	r[ 0] = 1.0 - y2 - z2;
	r[ 1] = xy + zw;
	r[ 2] = zx - yw;
	r[ 4] = xy - zw;
	r[ 5] = 1.0 - z2 - x2;
	r[ 6] = yz + xw;
	r[ 8] = zx + yw;
	r[ 9] = yz - xw;
	r[10] = 1.0 - x2 - y2;
	r[ 3] = r[ 7] = r[11] = r[12] = r[13] = r[14] = 0.0;
	r[15] = 1.0;
}

/*------------------ 
�t�s��擾
inMat:���͍s��i�P�����z��j
outMat:���͍s��̋t�s��i�P�����z��j
n:�s��̍s�i��j��
�Q�l�Fhttp://imagingsolution.net/math/gauss-jordan-with-pivot/
-------------------*/
void MakeInverseMatrix(const double* inMat, double* outMat,const int n )
{
	double tmp;
	int i, j, k;
	double **a;
	double **inv_a;

	// �z��̓��I�m��
	a = (double**)malloc(sizeof(double*)*n);
	a[0] = (double*)malloc(sizeof(double)*(n*n));
	for( i=1; i<n; i++ ){
		a[i] = a[i-1] + n;
	}
	inv_a = (double**)malloc(sizeof(double*)*n);
	inv_a[0] = (double*)malloc(sizeof(double)*(n*n));
	for( i=1;i<n;i++){
		inv_a[i] = inv_a[i-1] + n;
	}
	// ���͍s��̃R�s�[
	for( j=0; j<n; j++ ){
		for( i=0; i<n; i++ ){
			a[j][i] = inMat[i + n*j];
		}
	}
	// n���̒P�ʍs��̐���
	for( j=0; j<n; j++ ){
		for( i=0; i<n; i++ ){
			inv_a[j][i] = ( i==j ) ? 1.0 : 0.0;
		}
	}
	// �s�{�b�g�I�����s����Gauss-Jordan�@
	for( k=0; k<n; k++){
		//�s�{�b�g�I�� k�sk��ڂ̗v�f�̐�Βl���ő��
		int max = k;
		for( j=k+1; j<n; j++){
			if( fabs(a[j][k]) > fabs(a[max][k]) ){
				max = j;
			}
		}
		// �s�̓���ւ�
		if( max != k ){
			for( i=0; i<n; i++ ){
				// ���͍s��
				tmp = a[max][i];
				a[max][i] = a[k][i];
				a[k][i] = tmp;
				// �P�ʍs��
				tmp = inv_a[max][i];
				inv_a[max][i] = inv_a[k][i];
				inv_a[k][i] = tmp;
			}
		}
		// k�sk��ڂ̗v�f��1�ɂȂ�悤��
		tmp = a[k][k];
		for(i=0;i<n;i++){
			a[k][i] /= tmp;
			inv_a[k][i] /= tmp;
		}
		// k�s�ڂ�k��ڈȊO�̗v�f��0�ɂȂ�悤��
		for( j=0;j<n;j++ ){
			if( j != k ){
				tmp = 	a[j][k] / a[k][k];
				for(i=0;i<n;i++){
					a[j][i] = a[j][i] - a[k][i] * tmp;
					inv_a[j][i] = inv_a[j][i] - inv_a[k][i] * tmp;
				}
			}
		}

	}
	// ���߂��t�s����o�͍s��ɃR�s�[
	for( j=0; j<n; j++ ){
		for( i=0; i<n; i++ ){
			outMat[ i+n*j ] = inv_a[j][i];
		}
	}

	// �m�ۂ����������̈�̉��
	free(a[0]);
	free(a);
	free(inv_a[0]);
	free(inv_a);
}

double myRand( const double* min, const double* max)
{
	return rand() / ( RAND_MAX + 1.0 ) * ( (*max) - (*min) ) + (*min);
}

void CountFPS( DWORD* in_fps )
{
	static DWORD before_time = timeGetTime(); // �ȑO�̎���
    DWORD        now_time    = timeGetTime(); // ���݂̎���
    static DWORD fps_ctr = 0;

    if(now_time - before_time >= 1000){
        // ������
        before_time = now_time;
        *in_fps = fps_ctr;
        fps_ctr = 0;
    }

    fps_ctr++;
}