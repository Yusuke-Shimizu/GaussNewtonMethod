/*

�g����

GaussNewton(&inPara, &exPara, scale, &data, thresh);
gn.play(&gn_exPara);
�Ə�����gn_exPara�ɓ��e�덷���12���R�x�̍s��(3x4)������܂�

inPara	: Mat(3x3) �����p�����[�^
exPara	: Mat(3x4) ���߂��O���p�����[�^(12���R�x)
scale	: double �X�P�[���t�@�N�^(����͎g���ĂȂ��̂œK���ł������ł�)
data	: vector<transPoint> �f�[�^���S�ē������x�N�g��
thresh	: double 臒l�i����͓K����0.00094�ł����C0.01�ȉ��̏����߂̒l�Ȃ�ok�ł��j

�ERANSAC���g���ꍇ
RANSAC�ŋ��߂��f�[�^���N���XtransPoint�ɕϊ����Cvector�ɒǉ�
����ō�����V����vector�z��𓯗l��
GaussNewton(&inPara, &exPara, scale, &data, thresh);
gn.play(&gn_exPara);
��ok�ł�

*/

#ifndef GAUSS_NEWTON_H
#define GAUSS_NEWTON_H

#include "myCV.h"
#include "transPoint.h"
#include <vector>

using namespace cv;

class GaussNewton{
private:
	Mat inPara;					// �����p�����[�^(3x4)
	Mat exPara6DOF;				// �O���p�����[�^(6DOF)(6x1)
	Mat exPara;					// �O���p�����[�^(12DOF)(3x4)
	double scale;				// �X�P�[���t�@�N�^

	vector<transPoint> data;	// �f�[�^�Z�b�g(N)
	Mat jacobian;				// ���R�r�A���s��(?x?)
	Mat gradient;				// ���z(axb)
	Mat hessian;				// �w�b�Z�s��(bxb)
	double thresh;				// 臒l

	Mat dTds;					// T��s�Ŕ��������s��(�a�s��)
public:
	GaussNewton(Mat *_inPara, Mat *_exPara, double _scale, vector<transPoint> *_data, double _thresh);
	void setJacobian(Mat *_jaco);
	//void setGradient(Mat *_jaco, Mat *e, Mat *_grad);
	void setGradient(Mat *_grad);
	void setGradient(Mat *s, Mat *_grad);
	void setHessian(Mat *_jaco, Mat *_hess);
	void setHessian2(Mat *s, Mat *_hess);
	void setExPara6DOF(void);	// 12���R�x����6���R�x�ɗ��Ƃ�
	void setExPara6DOF(Mat *_exPara6DOF);	// 12���R�x����6���R�x�ɗ��Ƃ�
	void set_dTds(void);
	void set_p(Mat *p, Mat *P);

	// get�֐�(���x�グ��Ȃ�Ԃ�lvoid�ň�����double*)
	double get_fx(void);
	double get_fy(void);
	double get_x0(void);
	double get_y0(void);
	double get_R11(void);
	double get_R12(void);
	double get_R13(void);
	double get_R21(void);
	double get_R22(void);
	double get_R23(void);
	double get_R31(void);
	double get_R32(void);
	double get_R33(void);
	double get_Tx(void);
	double get_Ty(void);
	double get_Tz(void);
	//double get_k1(void);
	//double get_k2(void);
	//double get_k3(void);
	double get_R(const int r, const int c);
	double get_Xw(const Mat * const m3D, const int num);
	//void get_exPara6DOF(Mat *m);

	void get_dudPc(Mat *dst, Mat *m2D);
	void get_dPcdT(Mat *dst, Mat *m3D);
	void get_duds(Mat *dst, const int data_num);

	void get_dFds(const int num, transPoint *tp, Mat *s, Mat *dst);
	void get_dFdk1(transPoint *tp, Mat *s, Mat *dst);
	void get_dFdk2(transPoint *tp, Mat *s, Mat *dst);
	void get_dFdk3(transPoint *tp, Mat *s, Mat *dst);
	void get_dFdTx(transPoint *tp, Mat *s, Mat *dst);
	void get_dFdTy(transPoint *tp, Mat *s, Mat *dst);
	void get_dFdTz(transPoint *tp, Mat *s, Mat *dst);

	// ���̑�
	void dFdTx(Mat *dst);
	void calc_ds(Mat *ds, Mat *hessian, Mat *gradient);
	void updata_exPara6DOF(Mat *ds);
	void updata_exPara12DOF(Mat *ds);
	double calc_err(vector<transPoint> *data, Mat *exPara);

	void play(Mat *out);
};

#endif