//
// Created by Kunat Pipatanakul on 3/31/16.
//

#ifndef IMAGESTITCHING_UTIL_H
#define IMAGESTITCHING_UTIL_H

#include "math.h"
#include <android/log.h>
#include <vector>
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;
Mat multiply(Mat a,Mat b);
void remap(Mat src,Mat &dst,Mat xmap,Mat ymap);
cv::Mat estimateRigidTransform( InputArray src1, InputArray src2, bool fullAffine ,vector<uchar> &ransac );
#endif //IMAGESTITCHING_UTIL_H
