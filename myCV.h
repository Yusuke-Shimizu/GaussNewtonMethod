#ifndef MY_CV_H
#define MY_CV_H

/*------------OpenCV��lib�ݒ�------------*/
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
//Debug���[�h�̏ꍇ
#pragma comment(lib,"opencv_core220d.lib")            // opencv_core
#pragma comment(lib,"opencv_imgproc220d.lib")        // opencv_imgproc
#pragma comment(lib,"opencv_highgui220d.lib")        // opencv_highgui
#pragma comment(lib,"opencv_objdetect220d.lib")    // opencv_objdetect
#else
//Release���[�h�̏ꍇ
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
//Debug���[�h�̏ꍇ
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
//Release���[�h�̏ꍇ
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
    //Debug���[�h�̏ꍇ
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
    //Release���[�h�̏ꍇ
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
�֐����FInitCamera
�����@�F�J����������
�����@�Fio_cap CAPTURE�N���X
�߂�l�F����:0 ���s:1
���l�@�F
------------------------------------*/
int InitCamera( cv::VideoCapture* in_cap,int in_width, int in_height );

/*-----------------------------------
�֐����FGetMatPointColorBGR
�����@�F�w�肵����f�ʒu�̐F�l�擾
�����@�F
�߂�l�F
���l�@�Fcv::Mat���� BGR
------------------------------------*/
void GetMatPointColorBGR( cv::Mat* mat, int col[3], int X, int Y );

/*-----------------------------------
�֐����FDrawMatPointColorBGR
�����@�F�w�肵����f�ʒu�ɐF�l��`��
�����@�F
�߂�l�F
���l�@�Fcv::Mat���� BGR
------------------------------------*/
void DrawMatPointColorBGR( cv::Mat* mat, int col[3], int X, int Y );

/*-----------------------------------
�֐����FDetectHumanFace
�����@�F��F��
�����@�F
�߂�l�F
���l�@�F
------------------------------------*/
int DetectHumanFace( cv::Mat* in_img, double* in_scale, std::string cascadeName,
					cv::Point* out_center, int* out_radius);

void GLTex2CVMat( char *data, cv::Mat* image );

void convertParameter12to6(const cv::Mat* const para_3x4, cv::Mat *para_6x1);

void convertParameter6to12(const cv::Mat* const para_6x1, cv::Mat *para_3x4);

void printVectorMat(vector<Mat> *vec);

#endif
/*-----------------------------------
�֐����F
�����@�F
�����@�F
�߂�l�F
���l�@�F
------------------------------------*/