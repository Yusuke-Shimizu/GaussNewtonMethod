#ifndef MY_CV_H
#define MY_CV_H

/*------------OpenCVのlib設定------------*/
//#define OPENCV_2_1
//#define OPENCV_2_2
//#define OPENCV_2_3
#define OPENCV_2_4

/***** OpenCV2.1 *****/
#ifdef OPENCV_2_1
#ifdef _DEBUG
#pragma comment( lib, "cv210d.lib" )
#pragma comment( lib, "cxcore210d.lib" )
#pragma comment( lib, "cvaux210d.lib" )
#pragma comment( lib, "highgui210d.lib" )
#else
#pragma comment( lib, "cv210.lib" )
#pragma comment( lib, "cxcore210.lib" )
#pragma comment( lib, "cvaux210.lib" )
#pragma comment( lib, "highgui210.lib" )
#endif
#endif

/***** OpenCV2.2 *****/
#ifdef OPENCV_2_2
#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"opencv_core220d.lib")            // opencv_core
#pragma comment(lib,"opencv_imgproc220d.lib")        // opencv_imgproc
#pragma comment(lib,"opencv_highgui220d.lib")        // opencv_highgui
#pragma comment(lib,"opencv_objdetect220d.lib")    // opencv_objdetect
#else
//Releaseモードの場合
#pragma comment(lib,"opencv_calib3d220.lib")
#pragma comment(lib,"opencv_contrib220.lib")
#pragma comment(lib,"opencv_features2d220.lib")
#pragma comment(lib,"opencv_ffmpeg220.lib")
#pragma comment(lib,"opencv_flann220.lib")
#pragma comment(lib,"opencv_legacy220.lib")
#pragma comment(lib,"opencv_ml220.lib")
#pragma comment(lib,"opencv_core220.lib")            // opencv_core
#pragma comment(lib,"opencv_imgproc220.lib")        // opencv_imgproc
#pragma comment(lib,"opencv_highgui220.lib")        // opencv_highgui
#pragma comment(lib,"opencv_objdetect220.lib")    // opencv_objdetect
#endif
#endif

/***** OpenCV2.3 *****/
#ifdef OPENCV_2_3
#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"opencv_calib3d231d.lib")
#pragma comment(lib,"opencv_contrib231d.lib")
#pragma comment(lib,"opencv_core231d.lib")
#pragma comment(lib,"opencv_features2d231d.lib")
#pragma comment(lib,"opencv_flann231d.lib")
#pragma comment(lib,"opencv_gpu231d.lib")
#pragma comment(lib,"opencv_highgui231d.lib")
#pragma comment(lib,"opencv_imgproc231d.lib")
#pragma comment(lib,"opencv_legacy231d.lib")
#pragma comment(lib,"opencv_ml231d.lib")
#pragma comment(lib,"opencv_objdetect231d.lib")
#pragma comment(lib,"opencv_ts231d.lib")
#pragma comment(lib,"opencv_video231d.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"opencv_calib3d231.lib")
#pragma comment(lib,"opencv_contrib231.lib")
#pragma comment(lib,"opencv_core231.lib")
#pragma comment(lib,"opencv_features2d231.lib")
#pragma comment(lib,"opencv_flann231.lib")
#pragma comment(lib,"opencv_gpu231.lib")
#pragma comment(lib,"opencv_highgui231.lib")
#pragma comment(lib,"opencv_imgproc231.lib")
#pragma comment(lib,"opencv_legacy231.lib")
#pragma comment(lib,"opencv_ml231.lib")
#pragma comment(lib,"opencv_objdetect231.lib")
#pragma comment(lib,"opencv_ts231.lib")
#pragma comment(lib,"opencv_video231.lib")
#endif
#endif

/***** OpenCV2.4 *****/
#ifdef OPENCV_2_4
#ifdef _DEBUG
    //Debugモードの場合
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_core240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_imgproc240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_highgui240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_objdetect240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_contrib240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_features2d240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_flann240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_gpu240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_haartraining_engined.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_legacy240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_ts240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_video240d.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_nonfree240.lib")
	#pragma comment ( lib , "D:\\opencv\\build\\x86\\vc10\\lib\\opencv_calib3d240d.lib" )
#else
    //Releaseモードの場合
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_core240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_imgproc240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_highgui240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_objdetect240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_contrib240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_features2d240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_flann240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_gpu240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_haartraining_engined.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_legacy240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_ts240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_video240.lib")
    #pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_nonfree240d.lib")
	#pragma comment(lib,"D:\\opencv\\build\\x86\\vc10\\lib\\opencv_calib3d240.lib")
#endif
#endif

/*--------------------------------------*/
#include <stdio.h>
#include <iostream>

/***** OpenCV2.1 *****/
#ifdef OPENCV_2_1
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif
/***** OpenCV2.2(2.3) *****/
#ifdef OPENCV_2_2
#include "opencv2\\opencv.hpp"
#endif
#ifdef OPENCV_2_3
#include "opencv2\\opencv.hpp"
#endif
#ifdef OPENCV_2_4
#include <opencv2\opencv.hpp>
#endif

#define _printMat(var) std::cout<<#var<<" = "<<(var)<<" ("<<(var).rows<<"x"<<(var).cols<<") ("<<&(var)<<") ("<<__FUNCTION__<<")"<<std::endl

using namespace cv;

/*-----------------------------------
関数名：InitCamera
処理　：カメラ初期化
引数　：io_cap CAPTUREクラス
戻り値：成功:0 失敗:1
備考　：
------------------------------------*/
int InitCamera( cv::VideoCapture* in_cap,int in_width, int in_height );

/*-----------------------------------
関数名：GetMatPointColorBGR
処理　：指定した画素位置の色値取得
引数　：
戻り値：
備考　：cv::Mat限定 BGR
------------------------------------*/
void GetMatPointColorBGR( cv::Mat* mat, int col[3], int X, int Y );

/*-----------------------------------
関数名：DrawMatPointColorBGR
処理　：指定した画素位置に色値を描画
引数　：
戻り値：
備考　：cv::Mat限定 BGR
------------------------------------*/
void DrawMatPointColorBGR( cv::Mat* mat, int col[3], int X, int Y );

/*-----------------------------------
関数名：DetectHumanFace
処理　：顔認識
引数　：
戻り値：
備考　：
------------------------------------*/
int DetectHumanFace( cv::Mat* in_img, double* in_scale, std::string cascadeName,
					cv::Point* out_center, int* out_radius);

void GLTex2CVMat( char *data, cv::Mat* image );

void convertParameter12to6(const cv::Mat* const para_3x4, cv::Mat *para_6x1);

void convertParameter6to12(const cv::Mat* const para_6x1, cv::Mat *para_3x4);

void printVectorMat(vector<Mat> *vec);

#endif
/*-----------------------------------
関数名：
処理　：
引数　：
戻り値：
備考　：
------------------------------------*/