#include "defination.h"

#include <opencv2/opencv.hpp>

#include <iostream>



// 读取配置文件
void read_param(const std::string file_path, ParamClb* pmc) {
  cv::FileStorage fs;
  fs.open(file_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "con't open file " << file_path << std::endl;
  }
  else {
    fs["pattern_height"] >> pmc->pattern_height;
    fs["pattern_width"] >> pmc->pattern_width;
    fs["squre_size"] >> pmc->squre_size;
    fs["img_path"] >> pmc->img_path;
    fs["img_format"] >> pmc->img_format;
    fs["img_num"] >> pmc->img_num;
  }
  fs.release();
};

// 读取内参
void read_intrinsic(const std::string file_path, ParamIntrinsic* intr) {
  cv::FileStorage fs;
  fs.open(file_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "con't open file " << file_path << std::endl;
  }
  else {
    fs["cameraMatrix"] >> intr->cameraMatrix;
    fs["distCoeffs"] >> intr->distCoeffs;
    fs["rotation"] >> intr->rotation;
    fs["translation"] >> intr->translation;
  }
  fs.release();
};

void show_img(const std::string win_name, const cv::Mat img, int delay) {
  cv::namedWindow(win_name, cv::WINDOW_NORMAL);
  cv::imshow(win_name, img);
  cv::waitKey(delay);
}

void init3DPoints(cv::Size boardSize, cv::Size squareSize,
  std::vector<cv::Point3f>* singlePatternPoint)
{
  for (int i = 0; i < boardSize.height; i++)
  {
    for (int j = 0; j < boardSize.width; j++)
    {
      cv::Point3f tempPoint;//单个角点的三维坐标
      tempPoint.x = float(i * squareSize.width);
      tempPoint.y = float(j * squareSize.height);
      tempPoint.z = 0;
      (*singlePatternPoint).push_back(tempPoint);
    }
  }
}

void calibration_intrinsic(std::string param_path) {
  ParamClb pmc;
  read_param(param_path, &pmc);
  std::cout << "img_num = " << pmc.img_num << std::endl;
  cv::Size patternSize(pmc.pattern_height, pmc.pattern_width);
  std::string img_pth;
  cv::Mat img, gray;
  std::vector<cv::Point2f> corners;
  std::vector<std::vector<cv::Point2f>> cornersSeq;//存储所有棋盘图角点的二维坐标
  for (int i = 0; i < pmc.img_num; i++) {
    img_pth = pmc.img_path + std::to_string(i) + pmc.img_format;
    std::cout << img_pth << std::endl;
    img = cv::imread(img_pth, cv::IMREAD_UNCHANGED);
    show_img("img", img, 1);

    if (3 == img.channels()) {
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    }



    bool found = cv::findChessboardCorners(gray, patternSize, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH |
      cv::CALIB_CB_NORMALIZE_IMAGE |
      cv::CALIB_CB_FAST_CHECK);
    if (!found) {
      std::cout << "Can't find chessboard corners." << std::endl;
    }
    else {
      cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS +
          cv::TermCriteria::MAX_ITER, 30, 0.1));
    }
    cornersSeq.push_back(corners);
    if (1 == img.channels()) {
      cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }

    cv::drawChessboardCorners(img, patternSize, corners, found);
    show_img("img", img, 1000);

  }
  //获取图像尺寸
  uint16_t imgWidth = img.size().width;
  uint16_t imgHeight = img.size().height;
  cv::Size imgSize = cv::Size(imgWidth, imgHeight);

  /**************************摄像机标定******************************/
  cv::Size squre_size = cv::Size(pmc.squre_size, pmc.squre_size);//棋盘格尺寸
  std::vector<std::vector<cv::Point3f>> object_points;//所有棋盘图像的角点三维坐标
  std::vector<int> pointCounts;

  //初始化单幅靶标图片的三维点
  std::vector<cv::Point3f> singlePatternPoints;
  init3DPoints(patternSize, squre_size, &singlePatternPoints);
  //初始化标定板上的三维坐标

  for (int n = 0; n < pmc.img_num; n++)
  {
    object_points.push_back(singlePatternPoints);
    pointCounts.push_back(patternSize.width * patternSize.height);
  }

  /***开始标定***/
  std::cout << "*****开始标定!******" << std::endl;
  //内参矩阵
  cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
  //摄像机的5个畸变系数：k1,k2,p1,p2,k3
  cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
  std::vector<cv::Vec3d> rotation;//旋转向量
  std::vector<cv::Vec3d> translation;//平移向量
  //std::vector<cv::Mat> rotation;
  int flags = 0;
  cv::calibrateCamera(object_points, cornersSeq, imgSize, cameraMatrix, distCoeffs,
    rotation, translation, flags);

  std::cout << "cameraMatrix = " << cameraMatrix << std::endl;
  std::cout << "distCoeffs = " << distCoeffs << std::endl;
  //cv::calibrationMatrixValues;
  //cv::calibrateCameraRO;
  cv::FileStorage fs;
  fs.open("calib_data/intrinsic.xml", cv::FileStorage::WRITE);
  fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs <<
    "rotation" << rotation << "translation" << translation;

  //fs << "SAVE_SOURCE_IMG" << save_source_img;
  fs.release();

}





int main()
{
  // 标定内参
  calibration_intrinsic("param_calib.json");


  // 去畸变
  //ParamIntrinsic intr;
  //read_intrinsic("calib_data/intrinsic.xml", &intr);
  //std::cout << intr.distCoeffs << std::endl;
  //
  //cv::Mat img=cv::imread("chess_testNP1/1.bmp", cv::IMREAD_UNCHANGED);
  //cv::Mat undist;
  //cv::undistort(img, undist, intr.cameraMatrix, intr.distCoeffs);
  //show_img("img", img, 0);
  //show_img("undist", undist, 0);
  //cv::imwrite("img_undist.png", undist);
}



