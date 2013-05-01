#include "myCV.h"


int InitCamera( cv::VideoCapture* in_cap,int in_width, int in_height )
{
	//�捞�T�C�Y�̐ݒ�
	in_cap->set(CV_CAP_PROP_FRAME_WIDTH, in_width);
	in_cap->set(CV_CAP_PROP_FRAME_HEIGHT, in_height);
	// �J�������I�[�v���ł������̊m�F
	if(!in_cap->isOpened()) return -1;

	return 0;
}

void GetMatPointColorBGR( cv::Mat* mat, int col[3], int X, int Y )
{
	int matStep = mat->step;
	int matChannels = mat->channels();

	col[0] = mat->data[ matStep * Y + matChannels * X ];
	col[1] = mat->data[ matStep * Y + matChannels * X + 1];
	col[2] = mat->data[ matStep * Y + matChannels * X + 2];
}

void DrawMatPointColorBGR( cv::Mat* mat, int col[3], int X, int Y )
{
	int matStep = mat->step;
	int matChannels = mat->channels();

	mat->data[ matStep * Y + matChannels * X ] = col[0];
	mat->data[ matStep * Y + matChannels * X + 1] = col[1];
	mat->data[ matStep * Y + matChannels * X + 2] = col[2];
}

int DetectHumanFace( cv::Mat* in_img, double* in_scale, std::string cascadeName,
					cv::Point* out_center, int* out_radius)
{
	cv::Mat gray, smallImg(cv::saturate_cast<int>(in_img->rows/(*in_scale)), cv::saturate_cast<int>(in_img->cols/(*in_scale)), CV_8UC1);
	// �O���[�X�P�[���摜�ɕϊ�
	cv::cvtColor(*in_img, gray, CV_BGR2GRAY);
	// �������ԒZ�k�̂��߂ɉ摜���k��
	cv::resize(gray, smallImg, smallImg.size(), 0, 0, cv::INTER_LINEAR);
	cv::equalizeHist( smallImg, smallImg);

	// ���ފ�̓ǂݍ���
	cv::CascadeClassifier cascade;
	if(!cascade.load(cascadeName))
		return -1;
	
	std::vector<cv::Rect> faces;
	/// �}���`�X�P�[���i��j�T��xo
	// �摜�C�o�͋�`�C�k���X�P�[���C�Œ��`���C�i�t���O�j�C�ŏ���`
	cascade.detectMultiScale(smallImg, faces, 1.1, 2,
		CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

	// ����
	for(std::vector<cv::Rect>::const_iterator r = faces.begin(); r != faces.end(); ++r) {
		out_center->x = cv::saturate_cast<int>((r->x + r->width*0.5)*(*in_scale));
		out_center->y = cv::saturate_cast<int>((r->y + r->height*0.5)*(*in_scale));
		*out_radius = cv::saturate_cast<int>((r->width + r->height)*0.25*(*in_scale));
	}
	return 0; 
}

void GLTex2CVMat( char *data, cv::Mat* image )
{
	memcpy( image->data, data, image->rows * image->cols * image->channels() );
	cv::cvtColor( *image, *image, CV_BGR2RGB );
	cv::flip( *image, *image , 0 );
}

// 12DOF => 6DOF
void convertParameter12to6(const Mat* const para_3x4, Mat *para_6x1){
	// �����ݒ�
	Mat R3x3 = Mat::zeros(3, 3, CV_64F);
	Mat R3x1 = Mat::zeros(3, 1, CV_64F);

	// 3x3��������肾��
	R3x3 = (*para_3x4)( cv::Rect(0, 0, 3, 3) ).clone();
	//double tmp1 = R3x3.at<double>(0, 0);
	
	// 9DOF -> 3DOF
	cv::Rodrigues(R3x3, R3x1);

	// �X�P�[���̌v�Z
	//cv::Rodrigues(R3x3, R3x1);
	//double tmp2 = R3x3.at<double>(0, 0);

	// �ϊ���̒l����
	for(int r = 0; r < 3; ++ r){
		para_6x1->at<double>(r    , 0) = R3x1.at<double>(r, 0);
		para_6x1->at<double>(r + 3, 0) = para_3x4->at<double>(r, 3);
	}
}

// 6DOF => 12DOF
void convertParameter6to12(const cv::Mat* const para_6x1, cv::Mat *para_3x4){
	// �����ݒ�
	Mat R3x3 = Mat::zeros(3, 3, CV_64F);
	Mat R3x1 = Mat::zeros(3, 1, CV_64F);

	// ��]���������o��
	R3x1 = (*para_6x1)( cv::Rect(0, 0, 1, 3) ).clone();

	// 9DOF -> 3DOF
	cv::Rodrigues(R3x1, R3x3);
	//_printMat(R3x3);

	// �ϊ���̒l����
	for(int r = 0; r < 3; ++ r){
		// ��]����
		for(int c = 0; c < 3; ++ c){
			para_3x4->at<double>(r, c) = R3x3.at<double>(r, c);
		}

		// ���i����
		para_3x4->at<double>(r, 3) = para_6x1->at<double>(r + 3, 0);
	}
}

void printVectorMat(vector<Mat> *vec){
	for(vector<Mat>::iterator itr = vec->begin(); itr != vec->end(); ++ itr){
		_printMat(*itr);
	}
}

/*-----------------------------------
�֐����F
�����@�F
�����@�F
�߂�l�F
���l�@�F
------------------------------------*/
