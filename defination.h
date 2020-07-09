#pragma once
#include <opencv2/opencv.hpp>
#include <string>

struct ParamClb {

	uint16_t pattern_height = 9;
	uint16_t pattern_width = 12;
	double squre_size = 25;
	std::string img_path = "chessboard/";
	std::string img_format = ".jpg";
	int img_num = 15;
};

struct ParamIntrinsic {

	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
	//摄像机的5个畸变系数：k1,k2,p1,p2,k3
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
	std::vector<cv::Vec3d> rotation = {};//旋转向量
	std::vector<cv::Vec3d> translation = {};//平移向量
};
