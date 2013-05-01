/*

使い方

GaussNewton(&inPara, &exPara, scale, &data, thresh);
gn.play(&gn_exPara);
と書けばgn_exParaに投影誤差後の12自由度の行列(3x4)が入ります

inPara	: Mat(3x3) 内部パラメータ
exPara	: Mat(3x4) 求めた外部パラメータ(12自由度)
scale	: double スケールファクタ(今回は使ってないので適当でもいいです)
data	: vector<transPoint> データが全て入ったベクトル
thresh	: double 閾値（今回は適当に0.00094ですが，0.01以下の小さめの値ならokです）

・RANSACを使う場合
RANSACで求めたデータをクラスtransPointに変換し，vectorに追加
それで作った新しいvector配列を同様に
GaussNewton(&inPara, &exPara, scale, &data, thresh);
gn.play(&gn_exPara);
でokです

*/

#ifndef GAUSS_NEWTON_H
#define GAUSS_NEWTON_H

#include "myCV.h"
#include "transPoint.h"
#include <vector>

using namespace cv;

class GaussNewton{
private:
	Mat inPara;					// 内部パラメータ(3x4)
	Mat exPara6DOF;				// 外部パラメータ(6DOF)(6x1)
	Mat exPara;					// 外部パラメータ(12DOF)(3x4)
	double scale;				// スケールファクタ

	vector<transPoint> data;	// データセット(N)
	Mat jacobian;				// ヤコビアン行列(?x?)
	Mat gradient;				// 勾配(axb)
	Mat hessian;				// ヘッセ行列(bxb)
	double thresh;				// 閾値

	Mat dTds;					// Tをsで微分した行列(疎行列)
public:
	GaussNewton(Mat *_inPara, Mat *_exPara, double _scale, vector<transPoint> *_data, double _thresh);
	void setJacobian(Mat *_jaco);
	//void setGradient(Mat *_jaco, Mat *e, Mat *_grad);
	void setGradient(Mat *_grad);
	void setGradient(Mat *s, Mat *_grad);
	void setHessian(Mat *_jaco, Mat *_hess);
	void setHessian2(Mat *s, Mat *_hess);
	void setExPara6DOF(void);	// 12自由度から6自由度に落とす
	void setExPara6DOF(Mat *_exPara6DOF);	// 12自由度から6自由度に落とす
	void set_dTds(void);
	void set_p(Mat *p, Mat *P);

	// get関数(速度上げるなら返り値voidで引数にdouble*)
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

	// その他
	void dFdTx(Mat *dst);
	void calc_ds(Mat *ds, Mat *hessian, Mat *gradient);
	void updata_exPara6DOF(Mat *ds);
	void updata_exPara12DOF(Mat *ds);
	double calc_err(vector<transPoint> *data, Mat *exPara);

	void play(Mat *out);
};

#endif