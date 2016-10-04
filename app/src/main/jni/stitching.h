#ifndef IMAGESTITCHING_STITCHING_H
#define IMAGESTITCHING_STITCHING_H

#include <android/log.h>
#include <jni.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <float.h>
#include <thread>
#include <ctime>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "BundleCeres.h"
#include "composer.h"
#include "matcher.h"
#include "util.h"
#define M_PI 3.14159265358979323846
#define HQ_COMPOSE
//Note10.1
//double work_scale = 0.8, seam_scale = 0.3, compose_scale = 1.0;
//Nexus5x
double work_scale = 0.4, seam_scale = 0.2, compose_scale = 1.0;
double tracking_scale = 0.2;
string result_name = "/mnt/sdcard/result.png";
int blend_type = Blender::NO;
#define TAG "NATIVE_DEBUG"
cv::FileStorage debug("/sdcard/stitch/debug.yml", cv::FileStorage::WRITE);

struct ImagePackage{
    String name;
    Mat image;
    Mat full_image;
    Size size;
    Size compose_size;
    Size full_size;
    Point corner;
    Point compose_corner;
    Mat mask_warped;
    Mat compose_mask_warped;
    Mat image_warped;
    Mat compose_image_warped;
    int done;
    ImageFeatures feature;
    CameraParams param;
};
float focal_divider = 3.45;
//Note10.1
//float camera_focal_x=1425.559961560,camera_focal_y=1425.559961560,camera_ppx=745.027494,camera_ppy=436.7257419;
//Nexus5x
float camera_focal_x=1468.8033741635331,camera_focal_y=1468.8033741635331,camera_ppx=556.62040323411281,camera_ppy=987.91495706781313;
int work_width = 0;
int work_height = 0;
vector<ImagePackage> images;
Mat stitching_descriptor;
std::vector<KeyPoint> stitiching_keypoint;
vector<vector<Point2f>> p2d;
vector<vector<Point3f>> p3d;

//No need to re-done
Ptr<Feature2D> detector = Algorithm::create<Feature2D>("Feature2D.ORB");
int detector_setup = 1;
extern "C" {

JNIEXPORT void JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_nativeAligning(JNIEnv*, jobject, jlong imgaddr,jlong glRotAddr,jlong retaddr);
JNIEXPORT void JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_nativeAddStitch(JNIEnv*, jobject, jlong imgaddr,jlong rotaddr);
JNIEXPORT int JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_nativeStitch(JNIEnv*, jobject,jlong retAddr,jlong areaAddr,jlong rotAddr,jlong refinedRotAddr,jlong roiAddr,jlong k_rinvAddr);
JNIEXPORT int JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_nativeKeyFrameSelection(JNIEnv *env, jobject,jfloatArray rotMat);


void tracking(jlong imgaddr,jlong glrotaddr,jlong glprojaddr,jlong retaddr);
int findNearest(int from,int to,std::vector<ImagePackage> images,Mat &inputR);
void findDescriptor(Mat img,std::vector<KeyPoint> &keypoints ,Mat &descriptor);
vector<Rect> findROI(float warped_image_scale, std::vector<ImagePackage> images);
inline void vectorMatrixMultiply(float vec[],float matrix[], float *out );
inline Point3f calc3DPosition(Point2f keyPoint,float multiply_aspect);
inline int glhProjectf(float objx, float objy, float objz, float *modelview, float *projection, int *viewport, float *windowCoordinate);
inline float calcDistance(float x1,float y1, float z1,float x2,float y2,float z2);
void printMatrix(Mat mat,string text);
}

using namespace std;
using namespace cv;
using namespace cv::detail;

#endif