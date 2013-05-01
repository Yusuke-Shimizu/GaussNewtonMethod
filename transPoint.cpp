//#include <opencv2\opencv.hpp>
#include "myCV.h"
#include "transPoint.h"
#include "common.h"

using namespace std;
using namespace cv;

// コンストラクタ
transPoint::transPoint(void){
	set();
}

transPoint::transPoint(Mat *_point3D, Mat *_point2D_level1){
	set(_point3D, _point2D_level1, _point2D_level1, _point2D_level1);
}

transPoint::transPoint(Mat *_point3D, Mat *_point2D_level1, Mat *_point2D_level2, Mat *_point2D_level3){
	set(_point3D, _point2D_level1, _point2D_level2, _point2D_level3);
}

// セッティングメソッド
void transPoint::set(void){
	point3D			= Mat::zeros(4, 1, CV_64F);
	point2D_level1	= Mat::zeros(3, 1, CV_64F);
	point2D_level2	= Mat::zeros(3, 1, CV_64F);
	point2D_level3	= Mat::zeros(3, 1, CV_64F);
}

void transPoint::set(Mat *_point3D, Mat *_point2D_level1, Mat *_point2D_level2, Mat *_point2D_level3){
	point3D			= _point3D->clone();
	point2D_level1	= _point2D_level1->clone();
	point2D_level2	= _point2D_level2->clone();
	point2D_level3	= _point2D_level3->clone();
}

// 取得メソッド
void transPoint::get(const int num, Mat *m){
	switch(num){
	case (0):
		getPoint3D(m);
		break;
	case (1):
		getPoint2DLevel1(m);
		break;
	case (2):
		getPoint2DLevel2(m);
		break;
	case (3):
		getPoint2DLevel3(m);
		break;
	default:
		ERROR_PRINT(num);
		exit(0);
	}
}

void transPoint::getPoint3D(Mat *m){
	*m = point3D;
}

void transPoint::getPoint2DLevel1(Mat *m){
	*m = point2D_level1;
}

void transPoint::getPoint2DLevel2(Mat *m){
	*m = point2D_level2;
}

void transPoint::getPoint2DLevel3(Mat *m){
	*m = point2D_level3;

}

void transPoint::getPoint2D(Mat *m, const int num){
	if(num == 1){
		getPoint2DLevel1(m);
	}else if(num == 2){
		getPoint2DLevel2(m);
	}else if(num == 3){
		getPoint2DLevel3(m);
	}else{
		ERROR_PRINT(num);
		exit(0);
	}
}

// 描画関数
void transPoint::print(void){
	cout << point3D << endl;
	cout << point2D_level1 << endl;
	cout << point2D_level2 << endl;
	cout << point2D_level3 << endl;
}
