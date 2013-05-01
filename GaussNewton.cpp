#include "GaussNewton.h"
#include "myCV.h"
#include <cmath>
#include "common.h"

using namespace std;
using namespace cv;

/**************************   �R���X�g���N�^  **************************/
GaussNewton::GaussNewton(Mat *_inPara, Mat *_exPara, double _scale, vector<transPoint> *_data, double _thresh){
	inPara = _inPara->clone();
	exPara = _exPara->clone();
	exPara6DOF = Mat::zeros(6, 1, CV_64F);
	setExPara6DOF(&exPara6DOF);
	scale = _scale;
	data = *_data;				// �v����
	thresh = _thresh;

	set_dTds();		// dTds�̏����ݒ�

	jacobian = Mat::zeros(2, 6, CV_64F);
	gradient = Mat::zeros(6, 1, CV_64F);
	hessian  = Mat::zeros(6, 6, CV_64F);
}

/**************************   set�֐�  **************************/
// ���R�r�A���i�P�������j�̐���
void GaussNewton::setJacobian(Mat *_jaco){

}

// ���z�̐���
// g = J^T * e
//void GaussNewton::setGradient(Mat *_jaco, Mat *_e, Mat *_grad){
//	*_grad = _jaco->t() * (*_e);
//}
void GaussNewton::setGradient(Mat *s, Mat *_grad){
	// p����������z��̏�����
	Mat ds[6];
	for(int i = 0; i < 6; ++ i) ds[i] = Mat::zeros(3, 1, CV_64F);

	// �f�[�^��S�Č���
	vector<transPoint>::iterator itr;
	for(itr = data.begin(); itr != data.end(); ++ itr){
		// ������
		Mat calc_p	= Mat::zeros(3, 1, CV_64F);
		Mat ans_p	= Mat::zeros(3, 1, CV_64F);
		Mat diff_p	= Mat::zeros(3, 1, CV_64F);
		Mat P		= Mat::zeros(4, 1, CV_64F);

		// �f�[�^���Z�b�g(F = x-x')
		itr->getPoint2D(&ans_p, 1);
		itr->getPoint3D(&P);
		set_p(&calc_p, &P);
		diff_p = calc_p - ans_p;

		// s�̔����s��̃Z�b�g
		for(int i = 0; i < 6; ++ i){
			get_dFds(i, &(*itr), s, &ds[i]);
		}

		// 
		// p23�̃{�b�N�X�̉���
		for(int r = 0; r < 6; ++ r){
			// F^T * F'
			// 1x1�ɂȂ�ׁC�X�J���ʂɕϊ�
			Mat m = diff_p.t() * ds[r];
			double d = m.at<double>(0, 0);

			// �S�đ����Ă��Ă���
			_grad->at<double>(r, 0) += d;
		}
	}

	//_printMat(*_grad); cout << endl;
}

// �w�b�Z�s��(2������)�̐���
// H = J^T * J
void GaussNewton::setHessian(Mat *_jaco, Mat *_hess){
	//*_hess = _jaco->t() * (*_jaco);
	*_hess = (*_jaco) * _jaco->t();
}
void GaussNewton::setHessian2(Mat *s, Mat *_hess){
	// p����������z��̏�����
	Mat ds[6];
	for(int i = 0; i < 6; ++ i) ds[i] = Mat::zeros(3, 1, CV_64F);

	// �f�[�^��S�Č���
	vector<transPoint>::iterator itr;
	for(itr = data.begin(); itr != data.end(); ++ itr){
		// s�̔����s��̃Z�b�g
		for(int i = 0; i < 6; ++ i){
			get_dFds(i, &(*itr), s, &ds[i]);
		}

		// 
		// p23�̃{�b�N�X�̉���
		for(int r = 0; r < 6; ++ r){
			for(int c = 0; c < 6; ++ c){
				// F^T * F'
				// 1x1�ɂȂ�ׁC�X�J���ʂɕϊ�
				Mat m = ds[r].t() * ds[c];
				double d = m.at<double>(0, 0);

				// �S�đ����Ă��Ă���
				_hess->at<double>(r, c) += d;
			}
		}
	}
}

// 12���R�x����6���R�x�ɗ��Ƃ�
void GaussNewton::setExPara6DOF(void){
	convertParameter12to6(&exPara, &exPara6DOF);
}
void GaussNewton::setExPara6DOF(Mat *_exPara6DOF){
	convertParameter12to6(&exPara, _exPara6DOF);
}


// dTds�̐ݒ�
void GaussNewton::set_dTds(void){
	dTds = Mat::zeros(12, 6, CV_64F);
	dTds.at<double>( 1, 2) = -1;
	dTds.at<double>( 2, 1) =  1;
	dTds.at<double>( 3, 2) =  1;
	dTds.at<double>( 5, 0) = -1;
	dTds.at<double>( 6, 1) = -1;
	dTds.at<double>( 7, 0) =  1;
	dTds.at<double>( 9, 3) =  1;
	dTds.at<double>(10, 4) =  1;
	dTds.at<double>(11, 5) =  1;
}

// p=KRtp'��p�̐ݒ�
void GaussNewton::set_p(Mat *p, Mat *P){
	*p = inPara * exPara * (*P);
	*p /= p->at<double>(2, 0);
}



/**************************   get�֐�  **************************/
double GaussNewton::get_fx(void){ return inPara.at<double>(0, 0); }
double GaussNewton::get_fy(void){ return inPara.at<double>(1, 1); }
double GaussNewton::get_x0(void){ return inPara.at<double>(0, 2); }
double GaussNewton::get_y0(void){ return inPara.at<double>(1, 2); }

double GaussNewton::get_R11(void){ return exPara.at<double>(0, 0); }
double GaussNewton::get_R12(void){ return exPara.at<double>(0, 1); }
double GaussNewton::get_R13(void){ return exPara.at<double>(0, 2); }
double GaussNewton::get_R21(void){ return exPara.at<double>(1, 0); }
double GaussNewton::get_R22(void){ return exPara.at<double>(1, 1); }
double GaussNewton::get_R23(void){ return exPara.at<double>(1, 2); }
double GaussNewton::get_R31(void){ return exPara.at<double>(2, 0); }
double GaussNewton::get_R32(void){ return exPara.at<double>(2, 1); }
double GaussNewton::get_R33(void){ return exPara.at<double>(2, 2); }

double GaussNewton::get_Tx(void){ return exPara.at<double>(0, 3); }
double GaussNewton::get_Ty(void){ return exPara.at<double>(1, 3); }
double GaussNewton::get_Tz(void){ return exPara.at<double>(2, 3); }
//double GaussNewton::get_k1(void){ return exPara6DOF.at<double>(0, 0); }
//double GaussNewton::get_k2(void){ return exPara6DOF.at<double>(1, 0); }
//double GaussNewton::get_k3(void){ return exPara6DOF.at<double>(2, 0); }
double GaussNewton::get_R(const int r, const int c){ return exPara.at<double>(r, c); }	// ��]�s��̗v�f���擾
double GaussNewton::get_Xw(const Mat * const m3D, const int num){ return m3D->at<double>(num, 0); }
//void GaussNewton::get_exPara6DOF(Mat *m){ *m = exPara6DOF; }

// 2x3
// p.27�Q��
void GaussNewton::get_dudPc(Mat *dst, Mat *m2D){
	// �����ݒ�
	*dst = Mat::zeros(2, 3, CV_64F);

	// Xc�̐ݒ�
	Mat Xc = Mat::zeros(3, 1, CV_64F);
	Xc = scale * inPara.inv() * (*m2D);

	// �l�̑��
	dst->at<double>(0, 0) = get_fx() / Xc.at<double>(2, 0);
	dst->at<double>(1, 1) = get_fy() / Xc.at<double>(2, 0);
	dst->at<double>(0, 2) = (-1) * get_fx() / (Xc.at<double>(2, 0) * Xc.at<double>(2, 0));
	dst->at<double>(1, 2) = (-1) * get_fy() / (Xc.at<double>(2, 0) * Xc.at<double>(2, 0));
}

// 3x12
// p.29�Q��
void GaussNewton::get_dPcdT(Mat *dst, Mat *m3D){
	// ������(���܂�D�܂����Ȃ�)
	*dst = Mat::zeros(3, 12, CV_64F);

	// �l�̑��
	for(int r = 0; r < 3; ++ r){
		for(int c = 0; c < 3 * 3; ++ c){
			dst->at<double>(r, c) = get_R(r, c / 3) * get_Xw(m3D, c % 3);
		}
	}
	for(int r = 0; r < 3; ++ r){
		for(int c = 9; c < 12; ++ c){
			dst->at<double>(r, c) = get_R(r, c - 9);
		}
	}
	//_print(exPara);
	//_print(*m3D);
	//_print(*dst);
}

// 2x6
// u��s�Ŕ��������s����擾
void GaussNewton::get_duds(Mat *dst, const int data_num){
	// ���݂̍��W�l���擾
	Mat m3D, m2D;
	data.at(data_num).getPoint3D(&m3D);
	data.at(data_num).getPoint2D(&m2D, 1);	// level1�̑I��

	// �e�Δ����s����擾
	Mat dudPc, dPcdT;
	get_dudPc(&dudPc, &m2D);
	get_dPcdT(&dPcdT, &m3D);
	//_printMat(dudPc);
	//_printMat(dPcdT);
	//_printMat(dTds);

	// �e�Δ����s�񂩂�duds�̎擾
	*dst = dudPc * dPcdT * dTds;
}

// p23��dFds���擾
void GaussNewton::get_dFds(const int num, transPoint *tp, Mat *s, Mat *dst){
	switch(num){
	case(0):
		get_dFdk1(tp, s, dst);
		break;
	case(1):
		get_dFdk2(tp, s, dst);
		break;
	case(2):
		get_dFdk3(tp, s, dst);
		break;
	case(3):
		get_dFdTx(tp, s, dst);
		break;
	case(4):
		get_dFdTy(tp, s, dst);
		break;
	case(5):
		get_dFdTz(tp, s, dst);
		break;
	default:
		ERROR_PRINT(num);
		exit(0);
	};
}

/**************************   ���̑��̊֐�  **************************/
// �f���^s�̌v�Z
void GaussNewton::calc_ds(Mat *ds, Mat *hessian, Mat *gradient){
	*ds = -1 * hessian->inv() * (*gradient);
	//_print(hessian->inv());
	//_print(hessian->inv() * (*gradient));
}

// exPara6DOF�̍X�V
void GaussNewton::updata_exPara6DOF(Mat *ds){
	exPara6DOF += (*ds);
}

// exPara12DOF�̍X�V
void GaussNewton::updata_exPara12DOF(Mat *s){
	convertParameter6to12(s, &exPara);
}

// ���e�덷���v�Z
double GaussNewton::calc_err(vector<transPoint> *_data, Mat *_exPara){
	double err = 0.0;

	for(vector<transPoint>::iterator itr = _data->begin(); itr != _data->end(); ++ itr){
		// ������
		Mat m2D = Mat::zeros(3, 1, CV_64F), m3D = Mat::zeros(4, 1, CV_64F);
		Mat calc2D = Mat::zeros(3, 1, CV_64F);

		// ���W�̎擾
		itr->getPoint2D(&m2D, 1);
		itr->getPoint3D(&m3D);
		set_p(&calc2D, &m3D);

		// �덷���v�Z
		Mat diff = Mat::zeros(3, 1, CV_64F);
		diff = m2D - calc2D;
		double diffLength = cv::norm(diff);
		_printMat(diff);
		_printMat(m2D);
		_printMat(calc2D);
		cout << endl;
		//double error = sqrt( pow( calc2D.at<double>(0,0) - m2D.at<double>(0,0), 2.0 ) + 
		//	pow( calc2D.at<double>(1,0) - m2D.at<double>(1,0), 2.0 ) );
		//cout << error<< ", ";

		// �덷�����v���Ă���
		err += diffLength;
	}

	return err;
}


// �K�E�X�E�j���[�g���@���s��
void GaussNewton::play(Mat *out){
	// �����ݒ�
	double ds_norm = 10000.0;

	// s�̏����ݒ�
	Mat s = Mat::zeros(6, 1, CV_64F);
	setExPara6DOF(&s);
	// exPara6DOF�̏����ݒ�
	//setExPara6DOF();

	for(int k = 0; ds_norm > thresh; ++ k){
		// ���z�ƃw�b�Z�s��̐���
		Mat jaco = Mat::zeros(2, 6, CV_64F);
		Mat grad = Mat::zeros(6, 1, CV_64F);
		Mat hess = Mat::zeros(6, 6, CV_64F);
		setGradient(&s, &grad);
		setHessian2(&s, &hess);
		//_print(grad);
		//_print(hess);

		// s�̔����ړ���(�f���^s)���v�Z����
		Mat ds = Mat::zeros(4, 1, CV_64F);
		calc_ds(&ds, &hess, &grad);

		// s���X�V����
		s += ds;

		// exPara�̍X�V
		updata_exPara12DOF(&s);

		// norm���v�Z����
		ds_norm = cv::norm(ds);

		// �o�͕�
		_printMat(s);
		_printMat(ds);
		Mat Rt = Mat::zeros(3, 4, CV_64F);
		convertParameter6to12(&s, &Rt);
		//_printMat(Rt);
		_print(ds_norm);
		//cout << endl;
		double err = calc_err(&data, &Rt);
		_print(err);
		cout << endl;

		if(k > 20) break;
	}

	// ���ʂ�n��
	convertParameter6to12(&s, out);
}


// F��k1�Ŕ��������s����擾
void GaussNewton::get_dFdk1(transPoint *tp, Mat *s, Mat *dst){
	// �Ή��_�̍��W���擾
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// �����ݒ�(wxMaxima�ɍ��킹���)
	double k1 = s->at<double>(0, 0);
	double k2 = s->at<double>(1, 0);
	double k3 = s->at<double>(2, 0);
	double Tx = s->at<double>(3, 0);
	double Ty = s->at<double>(4, 0);
	double Tz = s->at<double>(5, 0);
	double fx = get_fx();
	double fy = get_fy();
	double x0 = get_x0();
	double y0 = get_y0();
	double Xw = m3D.at<double>(0, 0);
	double Yw = m3D.at<double>(1, 0);
	double Zw = m3D.at<double>(2, 0);
	double k_pow = pow(k1, 2) + pow(k2, 2) + pow(k3, 2);
	double theta = sqrt(k_pow);

	dst->at<double>(0, 0) = pow(((pow(k2,4)+2*pow(k1,2)*pow

		(k2,2)+pow(k1,4))*theta*
		pow(cos(theta),2)+(2*pow

		(k2,2)+2*pow(k1,2))*pow(k3,2)*theta*
		cos(theta)+pow(k3,4)

		*theta)*pow(Zw,2)+((((2*k1*pow(k2,2)+2*pow(k1,3))*
		pow

		(theta,2)*cos(theta)+2*k1*pow(k3,2)*pow(theta,2))*sin

		(theta)+(-2*
		pow(k2,3)-2*pow(k1,2)*k2)*k3*theta*pow(cos

		(theta),2)+((2*pow(k2,3)+2*
		pow(k1,2)*k2)*k3-2*k2*pow

		(k3,3))*theta*cos(theta)+2*k2*pow(k3,3)*theta)*
		Yw+(((-

		2*pow(k2,3)-2*pow(k1,2)*k2)*pow(theta,2)*cos(theta)-2*k2*pow

		(k3,2)*
		pow(theta,2))*sin(theta)+(-2*k1*pow(k2,2)-2*pow

		(k1,3))*k3*theta*
		pow(cos(theta),2)+((2*k1*pow(k2,2)+2*pow

		(k1,3))*k3-2*k1*pow(k3,3))*theta*
		cos(theta)+2*k1*pow

		(k3,3)*theta)*Xw+((2*pow(k2,2)+2*pow(k1,2))*k_pow*theta*


		cos(theta)+2*pow(k3,2)*k_pow*theta)*Tz)*Zw+(pow(k1,2)*pow

		(theta,3)*
		pow(sin(theta),2)+(2*k1*k2*k3*pow(theta,2)-

		2*k1*k2*k3*pow(theta,2)*
		cos(theta))*sin(theta)+pow(k2,2)

		*pow(k3,2)*theta*pow(cos(theta),2)-2*
		pow(k2,2)*pow(k3,2)

		*theta*cos(theta)+pow(k2,2)*pow(k3,2)*theta)*
		pow

		(Yw,2)+((-2*k1*k2*pow(theta,3)*pow(sin(theta),2)+((2*pow

		(k2,2)-2*
		pow(k1,2))*k3*pow(theta,2)*cos(theta)+(2*pow

		(k1,2)-2*pow(k2,2))*k3*
		pow(theta,2))*sin

		(theta)+2*k1*k2*pow(k3,2)*theta*pow(cos(theta),2)-4*k1*k2*


		pow(k3,2)*theta*cos(theta)+2*k1*k2*pow(k3,2)*theta)*Xw

		+(2*k1*k_pow*
		pow(theta,2)*sin(theta)-

		2*k2*k3*k_pow*theta*cos(theta)+2*k2*k3*k_pow*theta)*
		Tz)

		*Yw+(pow(k2,2)*pow(theta,3)*pow(sin(theta),2)+(2*k1*k2*k3*pow

		(theta,2)*
		cos(theta)-2*k1*k2*k3*pow(theta,2))*sin

		(theta)+pow(k1,2)*pow(k3,2)*theta*
		pow(cos(theta),2)-2*pow

		(k1,2)*pow(k3,2)*theta*cos(theta)+pow(k1,2)*
		pow(k3,2)

		*theta)*pow(Xw,2)+(-2*k2*k_pow*pow(theta,2)*sin(theta)-

		2*k1*k3*
		k_pow*theta*cos(theta)+2*k1*k3*k_pow*theta)*Tz*Xw

		+pow(k_pow,2)*theta*
		pow(Tz,2),-1)*(((fx*k1*pow

		(k2,3)+fx*pow(k1,3)*k2)*theta*
		pow(sin(theta),2)+((-

		2*fx*k1*k2*pow(theta,2)+fx*k1*pow(k2,3)+fx*pow(k1,3)*
		k2)

		*cos(theta)+fx*pow(k1,2)*pow(k3,3)+fx*k1*k2*pow(k3,2)+(fx*pow

		(k1,2)*
		pow(k2,2)+fx*pow(k1,4))*k3)*sin(theta)+((fx*pow

		(k1,2)-fx*pow(k2,2))*k3+fx*
		k1*pow(k2,3)+fx*pow(k1,3)*k2)

		*theta*pow(cos(theta),2)+(-fx*pow(k3,3)+fx*k1*
		k2*pow

		(k3,2)+(fx*pow(k2,2)-fx*pow(k1,2))*k3)*theta*cos

		(theta)+fx*pow(k3,3)*
		theta)*pow(Zw,2)+(((-fx*k2*pow

		(theta,3)-2*fx*k1*pow(k2,2)*k3*theta)*
		pow(sin

		(theta),2)+((2*fx*k1*k3*pow(theta,2)-2*fx*k1*pow(k2,2)*k3)*


		cos(theta)-fx*k1*pow(k3,3)+fx*pow(k1,2)*k2*pow

		(k3,2)+(fx*k1*pow(k2,2)-fx*
		pow(k1,3))*k3+fx*pow(k1,2)*pow

		(k2,3)+fx*pow(k1,4)*k2)*sin(theta)+(fx*k2*
		pow(k3,2)-

		2*fx*k1*pow(k2,2)*k3-fx*pow(k2,3)+fx*pow(k1,2)*k2)*theta*


		pow(cos(theta),2)+(-fx*k1*pow(k3,3)-3*fx*k2*pow

		(k3,2)+(fx*k1*pow(k2,2)-fx*
		pow(k1,3))*k3+fx*pow(k2,3)-

		fx*pow(k1,2)*k2)*theta*cos(theta)+2*fx*k2*
		pow(k3,2)

		*theta)*Yw+(-2*fx*pow(k1,2)*k2*k3*theta*pow(sin

		(theta),2)+((2*fx*
		k2*k3*pow(theta,2)-2*fx*pow(k1,2)

		*k2*k3)*cos(theta)-2*fx*k2*k3*
		pow(theta,2)-fx*k1*pow

		(k3,4)-fx*k1*pow(k2,2)*pow(k3,2)+2*fx*pow(k1,2)*k2*


		k3+fx*pow(k1,3)*pow(k2,2)+fx*pow(k1,5))*sin(theta)+(-

		2*fx*k1*pow(k3,2)-2*fx*
		pow(k1,2)*k2*k3-2*fx*k1*pow(k2,2))

		*theta*pow(cos(theta),2)+(2*fx*pow(k1,2)*


		k2*k3+2*fx*k1*pow(k2,2))*theta*cos(theta)+2*fx*k1*pow(k3,2)

		*theta)*Xw+(((fx*
		pow(k1,2)*k3+fx*k1*k2)*k_pow-

		2*fx*k1*k2*pow(theta,2))*sin(theta)+((fx*k1*
		k2-fx*k3)

		*k_pow+2*fx*pow(k1,2)*k3)*theta*cos(theta)+(fx*k3*k_pow-2*fx*


		pow(k1,2)*k3)*theta)*Tz+(((fx*k1*pow(k2,2)+fx*pow(k1,3))

		*pow(k3,2)+fx*k1*
		pow(k2,4)+2*fx*pow(k1,3)*pow

		(k2,2)+fx*pow(k1,5))*sin(theta)-2*fx*k1*
		pow(k3,2)

		*theta*cos(theta)+2*fx*k1*pow(k3,2)*theta)*Tx)*Zw+((fx*k3*


		pow(theta,3)+(fx*k1*k2*pow(k3,2)+fx*pow(k1,3)*k2)*theta)*


		pow(sin(theta),2)+((fx*k1*k2*pow(k3,2)+fx*pow(k1,3)*k2)*cos

		(theta)-fx*k1*k2*
		pow(k3,2)-fx*pow(k1,3)*k2)*sin

		(theta)+(fx*k1*k2*pow(k3,2)+fx*pow(k2,2)*
		k3+fx*pow(k1,3)

		*k2)*theta*pow(cos(theta),2)+(-fx*k1*k2*pow(k3,2)-2*fx*


		pow(k2,2)*k3-fx*pow(k1,3)*k2)*theta*cos(theta)+fx*pow(k2,2)

		*k3*theta)*
		pow(Yw,2)+((-2*fx*pow(k1,2)*pow(k2,2)

		*theta*pow(sin(theta),2)+((-2*fx*
		pow(k3,2)*pow(theta,2)-

		2*fx*pow(k1,2)*pow(k2,2))*cos(theta)+(fx*
		pow(k3,2)-fx*pow

		(k2,2)+fx*pow(k1,2))*pow(theta,2)-fx*k1*k2*pow(k3,3)-fx*


		pow(k1,2)*pow(k3,2)+(-fx*k1*pow(k2,3)-fx*pow(k1,3)*k2)

		*k3+fx*pow(k1,2)*
		pow(k2,2)-fx*pow(k1,4))*sin(theta)-

		2*fx*pow(k1,2)*pow(k2,2)*theta*
		pow(cos(theta),2)+(-fx*pow

		(k1,2)*pow(k3,2)-2*fx*k1*k2*k3+fx*pow(k1,2)*
		pow(k2,2)-

		fx*pow(k1,4))*theta*cos(theta)+2*fx*k1*k2*k3*theta)*Xw

		+((2*fx*k1*
		k3*pow(theta,2)+(fx*pow(k1,2)*k2-fx*k1*k3)

		*k_pow)*sin(theta)+((-fx*k1*k3-fx*
		k2)*k_pow+2*fx*pow

		(k1,2)*k2)*theta*cos(theta)+(fx*k2*k_pow-2*fx*pow(k1,2)*


		k2)*theta)*Tz+(((-fx*pow(k3,2)-fx*pow(k2,2)+fx*pow(k1,2))*pow

		(theta,2)-fx*
		k1*k2*pow(k3,3)-fx*pow(k1,2)*pow(k3,2)+(-

		fx*k1*pow(k2,3)-fx*pow(k1,3)*k2)*
		k3-fx*pow(k1,2)*pow

		(k2,2)-fx*pow(k1,4))*sin(theta)+(-fx*pow(k1,2)*
		pow(k3,2)-

		2*fx*k1*k2*k3-fx*pow(k1,2)*pow(k2,2)-fx*pow(k1,4))*theta*


		cos(theta)+2*fx*k1*k2*k3*theta)*Tx)*Yw+((fx*k1*k2*pow

		(k3,2)+fx*k1*
		pow(k2,3))*theta*pow(sin

		(theta),2)+((fx*k1*k2*pow(k3,2)+fx*k1*pow(k2,3))*
		cos

		(theta)-2*fx*k1*k2*pow(theta,2)-fx*pow(k1,2)*pow(k3,3)+(-

		fx*pow(k1,2)*
		pow(k2,2)-fx*pow(k1,4))*k3+fx*pow(k1,3)*k2)

		*sin(theta)+(fx*pow(k3,3)+fx*k1*
		k2*pow(k3,2)+fx*pow(k2,2)

		*k3+fx*k1*pow(k2,3))*theta*pow(cos(theta),2)+(-fx*
		pow

		(k3,3)+(-fx*pow(k2,2)-fx*pow(k1,2))*k3+fx*pow(k1,3)*k2)*theta*


		cos(theta)+fx*pow(k1,2)*k3*theta)*pow(Xw,2)+(((-fx*k1*pow

		(k3,2)-fx*k1*
		pow(k2,2))*k_pow*sin(theta)+(-2*fx*k1*pow

		(k3,2)-2*fx*k1*pow(k2,2))*theta*
		cos

		(theta)+(2*fx*k1*k_pow-2*fx*pow(k1,3))*theta)*Tz+((-

		2*fx*k1*k2*
		pow(theta,2)-fx*pow(k1,2)*pow

		(k3,3)+fx*k1*k2*pow(k3,2)+(-fx*pow(k1,2)*
		pow(k2,2)-fx*pow

		(k1,4))*k3+fx*k1*pow(k2,3)+fx*pow(k1,3)*k2)*sin(theta)+(fx*


		pow(k3,3)+fx*k1*k2*pow(k3,2)+(fx*pow(k2,2)-fx*pow(k1,2))

		*k3+fx*k1*
		pow(k2,3)+fx*pow(k1,3)*k2)*theta*cos

		(theta)+((fx*pow(k1,2)-fx*pow(k2,2))*
		k3-fx*pow(k3,3))

		*theta)*Tx)*Xw+(2*fx*k1*k_pow-2*fx*k1*pow(k3,2)-2*fx*k1*


		pow(k2,2)-2*fx*pow(k1,3))*theta*Tx*Tz);

	dst->at<double>(1, 0) = pow(((pow(k2,2)+pow(k1,2))*cos

		(theta)+pow(k3,2))*Zw+theta*(k1*
		sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*
		cos

		(theta))*Xw+k_pow*Tz,-2)*((2*k1*cos(theta)-k1*(pow(k2,2)+pow

		(k1,2))*
		pow(theta,-1)*sin(theta))*Zw+k1*pow(theta,-1)*

		(k1*sin(theta)*Yw-k2*
		sin(theta)*Xw)+theta*(sin(theta)*Yw

		+pow(k1,2)*pow(theta,-1)*cos(theta)*
		Yw-k1*k2*pow(theta,-

		1)*cos(theta)*Xw)+k1*k2*k3*pow(theta,-1)*sin(theta)*
		Yw

		+(pow(k1,2)*k3*pow(theta,-1)*sin(theta)-k3*cos(theta)+k3)*Xw

		+2*k1*Tz)*
		(theta*(fy*k1*sin(theta)*Zw-k1*sin(theta)*y0*Yw

		+k2*sin(theta)*Xw*y0-fy*k3*
		sin(theta)*Xw)+(((-pow(k2,2)-

		pow(k1,2))*cos(theta)-pow(k3,2))*y0+fy*k2*k3*
		cos(theta)-

		fy*k2*k3)*Zw+((k2*k3*cos(theta)-k2*k3)*y0+(-fy*pow(k3,2)-fy*


		pow(k1,2))*cos(theta)-fy*pow(k2,2))*Yw+((k1*k3*cos(theta)-

		k1*k3)*
		Xw+(-pow(k3,2)-pow(k2,2)-pow(k1,2))*Tz)

		*y0+(fy*k1*k2*cos(theta)-fy*k1*k2)*
		Xw+(-fy*pow(k3,2)-

		fy*pow(k2,2)-fy*pow(k1,2))*Ty)-pow(((pow(k2,2)+pow(k1,2))*


		cos(theta)+pow(k3,2))*Zw+theta*(k1*sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*
		k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*cos

		(theta))*Xw+k_pow*Tz,-1)*(k1*
		pow(theta,-1)*(fy*k1*sin

		(theta)*Zw-k1*sin(theta)*y0*Yw+k2*sin(theta)*Xw*
		y0-

		fy*k3*sin(theta)*Xw)+theta*(fy*sin(theta)*Zw+fy*pow(k1,2)*pow

		(theta,-1)*
		cos(theta)*Zw-sin(theta)*y0*Yw-pow(k1,2)*pow

		(theta,-1)*cos(theta)*y0*Yw+k1*
		k2*pow(theta,-1)*cos

		(theta)*Xw*y0-fy*k1*k3*pow(theta,-1)*cos(theta)*
		Xw)+((-

		k1*(-pow(k2,2)-pow(k1,2))*pow(theta,-1)*sin(theta)-2*k1*cos

		(theta))*
		y0-fy*k1*k2*k3*pow(theta,-1)*sin(theta))*Zw+(-

		k1*k2*k3*pow(theta,-1)*
		sin(theta)*y0-k1*(-fy*pow(k3,2)-

		fy*pow(k1,2))*pow(theta,-1)*sin(theta)-2*fy*
		k1*cos

		(theta))*Yw+((-pow(k1,2)*k3*pow(theta,-1)*sin(theta)+k3*


		cos(theta)-k3)*Xw-2*k1*Tz)*y0+(-fy*pow(k1,2)*k2*pow(theta,-1)

		*sin(theta)+fy*
		k2*cos(theta)-fy*k2)*Xw-2*fy*k1*Ty);

	dst->at<double>(2, 0) = 0;
}

// F��k2�Ŕ��������s����擾
void GaussNewton::get_dFdk2(transPoint *tp, Mat *s, Mat *dst){
	// �Ή��_�̍��W���擾
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// �����ݒ�(wxMaxima�ɍ��킹���)
	double k1 = s->at<double>(0, 0);
	double k2 = s->at<double>(1, 0);
	double k3 = s->at<double>(2, 0);
	double Tx = s->at<double>(3, 0);
	double Ty = s->at<double>(4, 0);
	double Tz = s->at<double>(5, 0);
	double fx = get_fx();
	double fy = get_fy();
	double x0 = get_x0();
	double y0 = get_y0();
	double Xw = m3D.at<double>(0, 0);
	double Yw = m3D.at<double>(1, 0);
	double Zw = m3D.at<double>(2, 0);
	double k_pow = pow(k1, 2) + pow(k2, 2) + pow(k3, 2);
	double theta = sqrt(k_pow);

	dst->at<double>(0, 0) = pow(((pow(k2,2)+pow(k1,2))*cos

		(theta)+pow(k3,2))*Zw+theta*(k1*
		sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*
		cos

		(theta))*Xw+k_pow*Tz,-1)*(k2*pow(theta,-1)*(fx*k2*sin(theta)

		*Zw+(k1*
		sin(theta)*x0-fx*k3*sin(theta))*Yw-k2*sin(theta)

		*x0*Xw)+theta*(fx*
		sin(theta)*Zw+fx*pow(k2,2)*pow(theta,-

		1)*cos(theta)*Zw+(k1*k2*pow(theta,-1)*
		cos(theta)*x0-

		fx*k2*k3*pow(theta,-1)*cos(theta))*Yw-sin(theta)*x0*
		Xw-

		pow(k2,2)*pow(theta,-1)*cos(theta)*x0*Xw)+((2*k2*cos(theta)-

		k2*
		(pow(k2,2)+pow(k1,2))*pow(theta,-1)*sin(theta))

		*x0+fx*k1*k2*k3*
		pow(theta,-1)*sin(theta))*Zw+((pow(k2,2)

		*k3*pow(theta,-1)*sin(theta)-k3*
		cos(theta)+k3)

		*x0+fx*k1*pow(k2,2)*pow(theta,-1)*sin(theta)-fx*k1*
		cos

		(theta)+fx*k1)*Yw+(k1*k2*k3*pow(theta,-1)*sin(theta)*x0-k2*

		(fx*
		pow(k3,2)+fx*pow(k2,2))*pow(theta,-1)*sin

		(theta)+2*fx*k2*cos(theta))*Xw+2*
		k2*Tz*x0+2*fx*k2*Tx)-pow

		(((pow(k2,2)+pow(k1,2))*cos(theta)+pow(k3,2))*
		Zw+theta*

		(k1*sin(theta)*Yw-k2*sin(theta)*Xw)+(k2*k3-k2*k3*cos(theta))*


		Yw+(k1*k3-k1*k3*cos(theta))*Xw+k_pow*Tz,-2)*((2*k2*cos

		(theta)-k2*
		(pow(k2,2)+pow(k1,2))*pow(theta,-1)*sin

		(theta))*Zw+k2*pow(theta,-1)*(k1*
		sin(theta)*Yw-k2*sin

		(theta)*Xw)+theta*(k1*k2*pow(theta,-1)*cos(theta)*
		Yw-sin

		(theta)*Xw-pow(k2,2)*pow(theta,-1)*cos(theta)*Xw)+(pow(k2,2)

		*k3*
		pow(theta,-1)*sin(theta)-k3*cos(theta)+k3)*Yw

		+k1*k2*k3*pow(theta,-1)*
		sin(theta)*Xw+2*k2*Tz)*(theta*

		(fx*k2*sin(theta)*Zw+(k1*sin(theta)*x0-fx*k3*
		sin(theta))

		*Yw-k2*sin(theta)*x0*Xw)+(((pow(k2,2)+pow(k1,2))*
		cos

		(theta)+pow(k3,2))*x0-fx*k1*k3*cos(theta)+fx*k1*k3)*Zw

		+((k2*k3-k2*k3*
		cos(theta))*x0-fx*k1*k2*cos

		(theta)+fx*k1*k2)*Yw+((k1*k3-k1*k3*cos(theta))*
		x0+(fx*pow

		(k3,2)+fx*pow(k2,2))*cos(theta)+fx*pow(k1,2))*Xw

		+k_pow*Tz*x0+(fx*
		pow(k3,2)+fx*pow(k2,2)+fx*pow(k1,2))*Tx);
	dst->at<double>(1, 0) = pow(((pow(k2,2)+pow(k1,2))*cos

		(theta)+pow(k3,2))*Zw+theta*(k1*
		sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*
		cos

		(theta))*Xw+k_pow*Tz,-2)*((2*k2*cos(theta)-k2*(pow(k2,2)+pow

		(k1,2))*
		pow(theta,-1)*sin(theta))*Zw+k2*pow(theta,-1)*

		(k1*sin(theta)*Yw-k2*
		sin(theta)*Xw)+theta*(k1*k2*pow

		(theta,-1)*cos(theta)*Yw-sin(theta)*
		Xw-pow(k2,2)*pow

		(theta,-1)*cos(theta)*Xw)+(pow(k2,2)*k3*pow(theta,-1)*
		sin

		(theta)-k3*cos(theta)+k3)*Yw+k1*k2*k3*pow(theta,-1)*sin

		(theta)*Xw+2*k2*
		Tz)*(theta*(fy*k1*sin(theta)*Zw-k1*sin

		(theta)*y0*Yw+k2*sin(theta)*Xw*y0-fy*
		k3*sin(theta)

		*Xw)+(((-pow(k2,2)-pow(k1,2))*cos(theta)-pow(k3,2))*y0+fy*k2*


		k3*cos(theta)-fy*k2*k3)*Zw+((k2*k3*cos(theta)-k2*k3)*y0+(-

		fy*pow(k3,2)-fy*
		pow(k1,2))*cos(theta)-fy*pow(k2,2))*Yw

		+((k1*k3*cos(theta)-k1*k3)*
		Xw+(-pow(k3,2)-pow(k2,2)-pow

		(k1,2))*Tz)*y0+(fy*k1*k2*cos(theta)-fy*k1*k2)*
		Xw+(-fy*pow

		(k3,2)-fy*pow(k2,2)-fy*pow(k1,2))*Ty)-pow(((pow(k2,2)+pow

		(k1,2))*
		cos(theta)+pow(k3,2))*Zw+theta*(k1*sin(theta)*Yw-

		k2*sin(theta)*Xw)+(k2*
		k3-k2*k3*cos(theta))*Yw+(k1*k3-

		k1*k3*cos(theta))*Xw+k_pow*Tz,-1)*(k2*
		pow(theta,-1)*

		(fy*k1*sin(theta)*Zw-k1*sin(theta)*y0*Yw+k2*sin(theta)*Xw*


		y0-fy*k3*sin(theta)*Xw)+theta*(fy*k1*k2*pow(theta,-1)*cos

		(theta)*Zw-k1*k2*
		pow(theta,-1)*cos(theta)*y0*Yw+sin

		(theta)*Xw*y0+pow(k2,2)*pow(theta,-1)*
		cos(theta)*Xw*y0-

		fy*k2*k3*pow(theta,-1)*cos(theta)*Xw)+((-k2*
		(-pow(k2,2)-

		pow(k1,2))*pow(theta,-1)*sin(theta)-2*k2*cos(theta))*y0-fy*


		pow(k2,2)*k3*pow(theta,-1)*sin(theta)+fy*k3*cos(theta)-fy*k3)*


		Zw+((-pow(k2,2)*k3*pow(theta,-1)*sin(theta)+k3*cos(theta)-

		k3)*y0-k2*(-fy*
		pow(k3,2)-fy*pow(k1,2))*pow(theta,-1)*sin

		(theta)-2*fy*k2)*Yw+(-k1*k2*k3*
		pow(theta,-1)*sin(theta)

		*Xw-2*k2*Tz)*y0+(-fy*k1*pow(k2,2)*pow(theta,-1)*
		sin

		(theta)+fy*k1*cos(theta)-fy*k1)*Xw-2*fy*k2*Ty);
	dst->at<double>(2, 0) = 0;
}

// F��k3�Ŕ��������s����擾
void GaussNewton::get_dFdk3(transPoint *tp, Mat *s, Mat *dst){
	// �Ή��_�̍��W���擾
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// �����ݒ�(wxMaxima�ɍ��킹���)
	double k1 = s->at<double>(0, 0);
	double k2 = s->at<double>(1, 0);
	double k3 = s->at<double>(2, 0);
	double Tx = s->at<double>(3, 0);
	double Ty = s->at<double>(4, 0);
	double Tz = s->at<double>(5, 0);
	double fx = get_fx();
	double fy = get_fy();
	double x0 = get_x0();
	double y0 = get_y0();
	double Xw = m3D.at<double>(0, 0);
	double Yw = m3D.at<double>(1, 0);
	double Zw = m3D.at<double>(2, 0);
	double k_pow = pow(k1, 2) + pow(k2, 2) + pow(k3, 2);
	double theta = sqrt(k_pow);

	dst->at<double>(0, 0) = pow(((pow(k2,2)+pow(k1,2))*cos

		(theta)+pow(k3,2))*Zw+theta*(k1*
		sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*
		cos

		(theta))*Xw+k_pow*Tz,-1)*(k3*pow(theta,-1)*(fx*k2*sin(theta)

		*Zw+(k1*
		sin(theta)*x0-fx*k3*sin(theta))*Yw-k2*sin(theta)

		*x0*Xw)+theta*(fx*k2*k3*
		pow(theta,-1)*cos(theta)*Zw

		+(k1*k3*pow(theta,-1)*cos(theta)*x0-fx*
		sin(theta)-fx*pow

		(k3,2)*pow(theta,-1)*cos(theta))*Yw-k2*k3*pow(theta,-1)*


		cos(theta)*x0*Xw)+((2*k3-(pow(k2,2)+pow(k1,2))*k3*pow(theta,-

		1)*sin(theta))*
		x0+fx*k1*pow(k3,2)*pow(theta,-1)*sin

		(theta)-fx*k1*cos(theta)+fx*k1)*Zw+((k2*
		pow(k3,2)*pow

		(theta,-1)*sin(theta)-k2*cos(theta)+k2)*x0+fx*k1*k2*k3*


		pow(theta,-1)*sin(theta))*Yw+((k1*pow(k3,2)*pow(theta,-1)*sin

		(theta)-k1*
		cos(theta)+k1)*x0-k3*(fx*pow(k3,2)+fx*pow

		(k2,2))*pow(theta,-1)*sin(theta)+2*
		fx*k3*cos(theta))*Xw

		+2*k3*Tz*x0+2*fx*k3*Tx)-pow(((pow(k2,2)+pow(k1,2))*
		cos

		(theta)+pow(k3,2))*Zw+theta*(k1*sin(theta)*Yw-k2*sin(theta)

		*Xw)+(k2*
		k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*cos(theta))

		*Xw+k_pow*Tz,-2)*((2*
		k3-(pow(k2,2)+pow(k1,2))*k3*pow

		(theta,-1)*sin(theta))*Zw+k3*pow(theta,-1)*
		(k1*sin(theta)

		*Yw-k2*sin(theta)*Xw)+theta*(k1*k3*pow(theta,-1)*cos(theta)*


		Yw-k2*k3*pow(theta,-1)*cos(theta)*Xw)+(k2*pow(k3,2)*pow

		(theta,-1)*
		sin(theta)-k2*cos(theta)+k2)*Yw+(k1*pow(k3,2)

		*pow(theta,-1)*sin(theta)-k1*
		cos(theta)+k1)*Xw+2*k3*Tz)*

		(theta*(fx*k2*sin(theta)*Zw+(k1*sin(theta)*x0-fx*
		k3*sin

		(theta))*Yw-k2*sin(theta)*x0*Xw)+(((pow(k2,2)+pow(k1,2))*


		cos(theta)+pow(k3,2))*x0-fx*k1*k3*cos(theta)+fx*k1*k3)*Zw

		+((k2*k3-k2*k3*
		cos(theta))*x0-fx*k1*k2*cos

		(theta)+fx*k1*k2)*Yw+((k1*k3-k1*k3*cos(theta))*
		x0+(fx*pow

		(k3,2)+fx*pow(k2,2))*cos(theta)+fx*pow(k1,2))*Xw

		+k_pow*Tz*x0+(fx*
		pow(k3,2)+fx*pow(k2,2)+fx*pow(k1,2))*Tx);
	dst->at<double>(1, 0) = pow(((pow(k2,2)+pow(k1,2))*cos

		(theta)+pow(k3,2))*Zw+theta*(k1*
		sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*
		cos

		(theta))*Xw+k_pow*Tz,-2)*((2*k3-(pow(k2,2)+pow(k1,2))*k3*pow

		(theta,-1)*
		sin(theta))*Zw+k3*pow(theta,-1)*(k1*sin(theta)

		*Yw-k2*sin(theta)*Xw)+theta*
		(k1*k3*pow(theta,-1)*cos

		(theta)*Yw-k2*k3*pow(theta,-1)*cos(theta)*Xw)+(k2*
		pow

		(k3,2)*pow(theta,-1)*sin(theta)-k2*cos(theta)+k2)*Yw+(k1*pow

		(k3,2)*
		pow(theta,-1)*sin(theta)-k1*cos(theta)+k1)*Xw

		+2*k3*Tz)*(theta*(fy*k1*
		sin(theta)*Zw-k1*sin(theta)*y0*Yw

		+k2*sin(theta)*Xw*y0-fy*k3*sin(theta)*
		Xw)+(((-pow(k2,2)-

		pow(k1,2))*cos(theta)-pow(k3,2))*y0+fy*k2*k3*
		cos(theta)-

		fy*k2*k3)*Zw+((k2*k3*cos(theta)-k2*k3)*y0+(-fy*pow(k3,2)-fy*


		pow(k1,2))*cos(theta)-fy*pow(k2,2))*Yw+((k1*k3*cos(theta)-

		k1*k3)*
		Xw+(-pow(k3,2)-pow(k2,2)-pow(k1,2))*Tz)

		*y0+(fy*k1*k2*cos(theta)-fy*k1*k2)*
		Xw+(-fy*pow(k3,2)-

		fy*pow(k2,2)-fy*pow(k1,2))*Ty)-pow(((pow(k2,2)+pow(k1,2))*


		cos(theta)+pow(k3,2))*Zw+theta*(k1*sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*
		k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*cos

		(theta))*Xw+k_pow*Tz,-1)*(k3*
		pow(theta,-1)*(fy*k1*sin

		(theta)*Zw-k1*sin(theta)*y0*Yw+k2*sin(theta)*Xw*
		y0-

		fy*k3*sin(theta)*Xw)+theta*(fy*k1*k3*pow(theta,-1)*cos(theta)

		*Zw-k1*k3*
		pow(theta,-1)*cos(theta)*y0*Yw+k2*k3*pow

		(theta,-1)*cos(theta)*Xw*y0-fy*
		sin(theta)*Xw-fy*pow(k3,2)

		*pow(theta,-1)*cos(theta)*
		Xw)+((-(-pow(k2,2)-pow(k1,2))

		*k3*pow(theta,-1)*sin(theta)-2*k3)*y0-fy*k2*
		pow(k3,2)*pow

		(theta,-1)*sin(theta)+fy*k2*cos(theta)-fy*k2)*Zw+((-k2*


		pow(k3,2)*pow(theta,-1)*sin(theta)+k2*cos(theta)-k2)*y0-k3*(-

		fy*
		pow(k3,2)-fy*pow(k1,2))*pow(theta,-1)*sin(theta)-

		2*fy*k3*cos(theta))*
		Yw+((-k1*pow(k3,2)*pow(theta,-1)*sin

		(theta)+k1*cos(theta)-k1)*Xw-2*k3*Tz)*
		y0-fy*k1*k2*k3*pow

		(theta,-1)*sin(theta)*Xw-2*fy*k3*Ty);

	dst->at<double>(2, 0) = 0;
}


// F��Tx�Ŕ��������s����擾
void GaussNewton::get_dFdTx(transPoint *tp, Mat *s, Mat *dst){
	// �Ή��_�̍��W���擾
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// �����ݒ�(wxMaxima�ɍ��킹���)
	double k1 = s->at<double>(0, 0);
	double k2 = s->at<double>(1, 0);
	double k3 = s->at<double>(2, 0);
	double Tx = s->at<double>(3, 0);
	double Ty = s->at<double>(4, 0);
	double Tz = s->at<double>(5, 0);
	double fx = get_fx();
	double fy = get_fy();
	double x0 = get_x0();
	double y0 = get_y0();
	double Xw = m3D.at<double>(0, 0);
	double Yw = m3D.at<double>(1, 0);
	double Zw = m3D.at<double>(2, 0);
	double k_pow = pow(k1, 2) + pow(k2, 2) + pow(k3, 2);
	double theta = sqrt(k_pow);

	dst->at<double>(0, 0) = (fx*pow(k3,2)+fx*pow(k2,2)+fx*pow(k1,2))*pow(((pow(k2,2)+pow(k1,2))*
		cos(theta)+pow(k3,2))*Zw+theta*(k1*sin(theta)*Yw-k2*sin(theta)*Xw)+(k2*
		k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*cos(theta))*Xw+k_pow*Tz,-1);
	dst->at<double>(1, 0) = 0;
	dst->at<double>(2, 0)= 0;
}

// F��Ty�Ŕ��������s����擾
void GaussNewton::get_dFdTy(transPoint *tp, Mat *s, Mat *dst){
	// �Ή��_�̍��W���擾
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// �����ݒ�(wxMaxima�ɍ��킹���)
	double k1 = s->at<double>(0, 0);
	double k2 = s->at<double>(1, 0);
	double k3 = s->at<double>(2, 0);
	double Tx = s->at<double>(3, 0);
	double Ty = s->at<double>(4, 0);
	double Tz = s->at<double>(5, 0);
	double fx = get_fx();
	double fy = get_fy();
	double x0 = get_x0();
	double y0 = get_y0();
	double Xw = m3D.at<double>(0, 0);
	double Yw = m3D.at<double>(1, 0);
	double Zw = m3D.at<double>(2, 0);
	double k_pow = pow(k1, 2) + pow(k2, 2) + pow(k3, 2);
	double theta = sqrt(k_pow);

	dst->at<double>(0, 0) = 0;
	dst->at<double>(1, 0) = (fy*pow(k3,2)+fy*pow(k2,2)+fy*pow(k1,2))*pow(((pow(k2,2)+pow(k1,2))*cos(theta)+pow(k3,2))*Zw+theta*(k1*sin(theta)*Yw-k2*sin(theta)*Xw)+(k2*k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*cos(theta))*Xw+k_pow*Tz,-1);
	dst->at<double>(2, 0) = 0;
}

// F��Tz�Ŕ��������s����擾
void GaussNewton::get_dFdTz(transPoint *tp, Mat *s, Mat *dst){
	// �Ή��_�̍��W���擾
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// �����ݒ�(wxMaxima�ɍ��킹���)
	double k1 = s->at<double>(0, 0);
	double k2 = s->at<double>(1, 0);
	double k3 = s->at<double>(2, 0);
	double Tx = s->at<double>(3, 0);
	double Ty = s->at<double>(4, 0);
	double Tz = s->at<double>(5, 0);
	double fx = get_fx();
	double fy = get_fy();
	double x0 = get_x0();
	double y0 = get_y0();
	double Xw = m3D.at<double>(0, 0);
	double Yw = m3D.at<double>(1, 0);
	double Zw = m3D.at<double>(2, 0);
	double k_pow = pow(k1, 2) + pow(k2, 2) + pow(k3, 2);
	double theta = sqrt(k_pow);

	dst->at<double>(0, 0) = k_pow*x0*pow(((pow(k2,2)+pow(k1,2))

		*cos(theta)+pow(k3,2))*Zw+theta*
		(k1*sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*
		k3*cos

		(theta))*Xw+k_pow*Tz,-1)-k_pow*pow(((pow(k2,2)+pow(k1,2))*


		cos(theta)+pow(k3,2))*Zw+theta*(k1*sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*
		k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*cos

		(theta))*Xw+k_pow*Tz,-2)*(theta*(fx*
		k2*sin(theta)*Zw

		+(k1*sin(theta)*x0-fx*k3*sin(theta))*Yw-k2*sin(theta)*x0*


		Xw)+(((pow(k2,2)+pow(k1,2))*cos(theta)+pow(k3,2))*x0-

		fx*k1*k3*cos(theta)+fx*
		k1*k3)*Zw+((k2*k3-k2*k3*cos

		(theta))*x0-fx*k1*k2*cos(theta)+fx*k1*k2)*
		Yw+((k1*k3-

		k1*k3*cos(theta))*x0+(fx*pow(k3,2)+fx*pow(k2,2))*cos

		(theta)+fx*
		pow(k1,2))*Xw+k_pow*Tz*x0+(fx*pow(k3,2)+fx*pow

		(k2,2)+fx*pow(k1,2))*Tx);
	dst->at<double>(1, 0) = k_pow*pow(((pow(k2,2)+pow(k1,2))*cos

		(theta)+pow(k3,2))*Zw+theta*(k1*
		sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*
		cos

		(theta))*Xw+k_pow*Tz,-2)*(theta*(fy*k1*sin(theta)*Zw-k1*sin

		(theta)*y0*
		Yw+k2*sin(theta)*Xw*y0-fy*k3*sin(theta)

		*Xw)+(((-pow(k2,2)-pow(k1,2))*
		cos(theta)-pow(k3,2))

		*y0+fy*k2*k3*cos(theta)-fy*k2*k3)*Zw+((k2*k3*
		cos(theta)-

		k2*k3)*y0+(-fy*pow(k3,2)-fy*pow(k1,2))*cos(theta)-fy*pow

		(k2,2))*
		Yw+((k1*k3*cos(theta)-k1*k3)*Xw+(-pow(k3,2)-pow

		(k2,2)-pow(k1,2))*Tz)*y0+(fy*
		k1*k2*cos(theta)-fy*k1*k2)

		*Xw+(-fy*pow(k3,2)-fy*pow(k2,2)-fy*pow(k1,2))*
		Ty)-(-pow

		(k3,2)-pow(k2,2)-pow(k1,2))*y0*pow(((pow(k2,2)+pow(k1,2))*


		cos(theta)+pow(k3,2))*Zw+theta*(k1*sin(theta)*Yw-k2*sin

		(theta)*Xw)+(k2*
		k3-k2*k3*cos(theta))*Yw+(k1*k3-k1*k3*cos

		(theta))*Xw+k_pow*Tz,-1);

	dst->at<double>(2, 0) = 0;
}
