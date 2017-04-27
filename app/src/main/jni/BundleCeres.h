//
// Created by Kunat Pipatanakul on 1/26/16 AD.
//

#ifndef IMAGESTITCHING_BUNDLECERES_H
#define IMAGESTITCHING_BUNDLECERES_H

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include "opencv2/calib3d/calib3d.hpp"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "cmath"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
using namespace std;
using namespace cv;
using namespace cv::detail;
int minimizeRotation(std::vector<ImageFeatures> features,vector<MatchesInfo> pairs,vector<CameraParams> &cameras);
int minimizeRotation(vector<Point2f> src,vector<Point2f> dst,vector<CameraParams> &cameras);

void test();


#endif //IMAGESTITCHING_BUNDLECERES_H
