//
// Created by kunato on 2/15/17.
//

#ifndef IMAGESTITCHING_BUNDLECV_H
#define IMAGESTITCHING_BUNDLECV_H
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
using namespace cv::detail;
using namespace std;
using namespace cv;

bool estimate_(vector<Point2f> src,vector<Point2f> dst,Size size,
               std::vector<CameraParams> &cameras,bool focal_estimate);
#endif //IMAGESTITCHING_BUNDLECV_H
