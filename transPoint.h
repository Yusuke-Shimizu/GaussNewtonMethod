#ifndef TRANS_POINT_H
#define TRANS_POINT_H

//#include <opencv2\opencv.hpp>
#include "myCV.h"
using namespace cv;

class transPoint{
private:
	Mat point3D;
	Mat point2D_level1;
	Mat point2D_level2;
	Mat point2D_level3;
public:
	transPoint(void);
	transPoint(Mat *_point3D, Mat *_point2D_level1);
	transPoint(Mat *_point3D, Mat *_point2D_level1, Mat *_point2D_level2, Mat *_point2D_level3);
	void set(void);
	void set(Mat *_point3D, Mat *_point2D_level1, Mat *_point2D_level2, Mat *_point2D_level3);
	void get(const int num, Mat *m);
	void getPoint3D(Mat *m);
	void getPoint2DLevel1(Mat *m);
	void getPoint2DLevel2(Mat *m);
	void getPoint2DLevel3(Mat *m);
	void getPoint2D(Mat *m, const int num);
	void print(void);
};

#endif