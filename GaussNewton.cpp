#include "GaussNewton.h"
#include "myCV.h"
#include <cmath>
#include "common.h"

using namespace std;
using namespace cv;

/**************************   コンストラクタ  **************************/
GaussNewton::GaussNewton(Mat *_inPara, Mat *_exPara, double _scale, vector<transPoint> *_data, double _thresh){
	inPara = _inPara->clone();
	exPara = _exPara->clone();
	exPara6DOF = Mat::zeros(6, 1, CV_64F);
	setExPara6DOF(&exPara6DOF);
	scale = _scale;
	data = *_data;				// 要注意
	thresh = _thresh;

	set_dTds();		// dTdsの初期設定

	jacobian = Mat::zeros(2, 6, CV_64F);
	gradient = Mat::zeros(6, 1, CV_64F);
	hessian  = Mat::zeros(6, 6, CV_64F);
}

/**************************   set関数  **************************/
// ヤコビアン（１次微分）の生成
void GaussNewton::setJacobian(Mat *_jaco){

}

// 勾配の生成
// g = J^T * e
//void GaussNewton::setGradient(Mat *_jaco, Mat *_e, Mat *_grad){
//	*_grad = _jaco->t() * (*_e);
//}
void GaussNewton::setGradient(Mat *s, Mat *_grad){
	// pを微分した配列の初期化
	Mat ds[6];
	for(int i = 0; i < 6; ++ i) ds[i] = Mat::zeros(3, 1, CV_64F);

	// データを全て見る
	vector<transPoint>::iterator itr;
	for(itr = data.begin(); itr != data.end(); ++ itr){
		// 初期化
		Mat calc_p	= Mat::zeros(3, 1, CV_64F);
		Mat ans_p	= Mat::zeros(3, 1, CV_64F);
		Mat diff_p	= Mat::zeros(3, 1, CV_64F);
		Mat P		= Mat::zeros(4, 1, CV_64F);

		// データをセット(F = x-x')
		itr->getPoint2D(&ans_p, 1);
		itr->getPoint3D(&P);
		set_p(&calc_p, &P);
		diff_p = calc_p - ans_p;

		// sの微分行列のセット
		for(int i = 0; i < 6; ++ i){
			get_dFds(i, &(*itr), s, &ds[i]);
		}

		// 
		// p23のボックスの下側
		for(int r = 0; r < 6; ++ r){
			// F^T * F'
			// 1x1になる為，スカラ量に変換
			Mat m = diff_p.t() * ds[r];
			double d = m.at<double>(0, 0);

			// 全て足してしていく
			_grad->at<double>(r, 0) += d;
		}
	}

	//_printMat(*_grad); cout << endl;
}

// ヘッセ行列(2次微分)の生成
// H = J^T * J
void GaussNewton::setHessian(Mat *_jaco, Mat *_hess){
	//*_hess = _jaco->t() * (*_jaco);
	*_hess = (*_jaco) * _jaco->t();
}
void GaussNewton::setHessian2(Mat *s, Mat *_hess){
	// pを微分した配列の初期化
	Mat ds[6];
	for(int i = 0; i < 6; ++ i) ds[i] = Mat::zeros(3, 1, CV_64F);

	// データを全て見る
	vector<transPoint>::iterator itr;
	for(itr = data.begin(); itr != data.end(); ++ itr){
		// sの微分行列のセット
		for(int i = 0; i < 6; ++ i){
			get_dFds(i, &(*itr), s, &ds[i]);
		}

		// 
		// p23のボックスの下側
		for(int r = 0; r < 6; ++ r){
			for(int c = 0; c < 6; ++ c){
				// F^T * F'
				// 1x1になる為，スカラ量に変換
				Mat m = ds[r].t() * ds[c];
				double d = m.at<double>(0, 0);

				// 全て足してしていく
				_hess->at<double>(r, c) += d;
			}
		}
	}
}

// 12自由度から6自由度に落とす
void GaussNewton::setExPara6DOF(void){
	convertParameter12to6(&exPara, &exPara6DOF);
}
void GaussNewton::setExPara6DOF(Mat *_exPara6DOF){
	convertParameter12to6(&exPara, _exPara6DOF);
}


// dTdsの設定
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

// p=KRtp'のpの設定
void GaussNewton::set_p(Mat *p, Mat *P){
	*p = inPara * exPara * (*P);
	*p /= p->at<double>(2, 0);
}



/**************************   get関数  **************************/
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
double GaussNewton::get_R(const int r, const int c){ return exPara.at<double>(r, c); }	// 回転行列の要素を取得
double GaussNewton::get_Xw(const Mat * const m3D, const int num){ return m3D->at<double>(num, 0); }
//void GaussNewton::get_exPara6DOF(Mat *m){ *m = exPara6DOF; }

// 2x3
// p.27参照
void GaussNewton::get_dudPc(Mat *dst, Mat *m2D){
	// 初期設定
	*dst = Mat::zeros(2, 3, CV_64F);

	// Xcの設定
	Mat Xc = Mat::zeros(3, 1, CV_64F);
	Xc = scale * inPara.inv() * (*m2D);

	// 値の代入
	dst->at<double>(0, 0) = get_fx() / Xc.at<double>(2, 0);
	dst->at<double>(1, 1) = get_fy() / Xc.at<double>(2, 0);
	dst->at<double>(0, 2) = (-1) * get_fx() / (Xc.at<double>(2, 0) * Xc.at<double>(2, 0));
	dst->at<double>(1, 2) = (-1) * get_fy() / (Xc.at<double>(2, 0) * Xc.at<double>(2, 0));
}

// 3x12
// p.29参照
void GaussNewton::get_dPcdT(Mat *dst, Mat *m3D){
	// 初期化(あまり好ましくない)
	*dst = Mat::zeros(3, 12, CV_64F);

	// 値の代入
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
// uをsで微分した行列を取得
void GaussNewton::get_duds(Mat *dst, const int data_num){
	// 現在の座標値を取得
	Mat m3D, m2D;
	data.at(data_num).getPoint3D(&m3D);
	data.at(data_num).getPoint2D(&m2D, 1);	// level1の選択

	// 各偏微分行列を取得
	Mat dudPc, dPcdT;
	get_dudPc(&dudPc, &m2D);
	get_dPcdT(&dPcdT, &m3D);
	//_printMat(dudPc);
	//_printMat(dPcdT);
	//_printMat(dTds);

	// 各偏微分行列からdudsの取得
	*dst = dudPc * dPcdT * dTds;
}

// p23のdFdsを取得
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

/**************************   その他の関数  **************************/
// デルタsの計算
void GaussNewton::calc_ds(Mat *ds, Mat *hessian, Mat *gradient){
	*ds = -1 * hessian->inv() * (*gradient);
	//_print(hessian->inv());
	//_print(hessian->inv() * (*gradient));
}

// exPara6DOFの更新
void GaussNewton::updata_exPara6DOF(Mat *ds){
	exPara6DOF += (*ds);
}

// exPara12DOFの更新
void GaussNewton::updata_exPara12DOF(Mat *s){
	convertParameter6to12(s, &exPara);
}

// 投影誤差を計算
double GaussNewton::calc_err(vector<transPoint> *_data, Mat *_exPara){
	double err = 0.0;

	for(vector<transPoint>::iterator itr = _data->begin(); itr != _data->end(); ++ itr){
		// 初期化
		Mat m2D = Mat::zeros(3, 1, CV_64F), m3D = Mat::zeros(4, 1, CV_64F);
		Mat calc2D = Mat::zeros(3, 1, CV_64F);

		// 座標の取得
		itr->getPoint2D(&m2D, 1);
		itr->getPoint3D(&m3D);
		set_p(&calc2D, &m3D);

		// 誤差を計算
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

		// 誤差を合計していく
		err += diffLength;
	}

	return err;
}


// ガウス・ニュートン法を行う
void GaussNewton::play(Mat *out){
	// 初期設定
	double ds_norm = 10000.0;

	// sの初期設定
	Mat s = Mat::zeros(6, 1, CV_64F);
	setExPara6DOF(&s);
	// exPara6DOFの初期設定
	//setExPara6DOF();

	for(int k = 0; ds_norm > thresh; ++ k){
		// 勾配とヘッセ行列の生成
		Mat jaco = Mat::zeros(2, 6, CV_64F);
		Mat grad = Mat::zeros(6, 1, CV_64F);
		Mat hess = Mat::zeros(6, 6, CV_64F);
		setGradient(&s, &grad);
		setHessian2(&s, &hess);
		//_print(grad);
		//_print(hess);

		// sの微小移動量(デルタs)を計算する
		Mat ds = Mat::zeros(4, 1, CV_64F);
		calc_ds(&ds, &hess, &grad);

		// sを更新する
		s += ds;

		// exParaの更新
		updata_exPara12DOF(&s);

		// normを計算する
		ds_norm = cv::norm(ds);

		// 出力部
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

	// 結果を渡す
	convertParameter6to12(&s, out);
}


// Fをk1で微分した行列を取得
void GaussNewton::get_dFdk1(transPoint *tp, Mat *s, Mat *dst){
	// 対応点の座標を取得
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// 初期設定(wxMaximaに合わせる為)
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

// Fをk2で微分した行列を取得
void GaussNewton::get_dFdk2(transPoint *tp, Mat *s, Mat *dst){
	// 対応点の座標を取得
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// 初期設定(wxMaximaに合わせる為)
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

// Fをk3で微分した行列を取得
void GaussNewton::get_dFdk3(transPoint *tp, Mat *s, Mat *dst){
	// 対応点の座標を取得
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// 初期設定(wxMaximaに合わせる為)
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


// FをTxで微分した行列を取得
void GaussNewton::get_dFdTx(transPoint *tp, Mat *s, Mat *dst){
	// 対応点の座標を取得
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// 初期設定(wxMaximaに合わせる為)
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

// FをTyで微分した行列を取得
void GaussNewton::get_dFdTy(transPoint *tp, Mat *s, Mat *dst){
	// 対応点の座標を取得
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// 初期設定(wxMaximaに合わせる為)
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

// FをTzで微分した行列を取得
void GaussNewton::get_dFdTz(transPoint *tp, Mat *s, Mat *dst){
	// 対応点の座標を取得
	Mat /*m2D = Mat::zeros(3, 1, CV_64F), */m3D = Mat::zeros(4, 1, CV_64F);
	//tp->getPoint2D(m2D, 1);
	tp->getPoint3D(&m3D);

	// 初期設定(wxMaximaに合わせる為)
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
