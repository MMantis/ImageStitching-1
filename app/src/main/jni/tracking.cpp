//
// Created by kunato on 1/23/17.
//
#ifndef IMAGESTITCHING_TRACKING_H
#define IMAGESTITCHING_TRACKING_H

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
#include "opencv2/stitching/detail/warpers.hpp"
#include <opencv2/video/video.hpp>
#include "BundleCeres.h"
#include "composer.h"
#include "matcher.h"
#include "util.h"
inline void vectorMatrixMultiply(float vec[],float matrix[], float *out ){
    out[0] = matrix[0] * vec[0] + matrix[1] * vec[1] + matrix[2] * vec[2];
    out[1] = matrix[3] * vec[0] + matrix[4] * vec[1] + matrix[5] * vec[2];
    out[2] = matrix[6] * vec[0] + matrix[7] * vec[1] + matrix[8] * vec[2];
}
void printMatrix(Mat tmp,string text){
    Mat mat;
    if(mat.type() != CV_32F){
        tmp.convertTo(mat,CV_32F);
    }
    __android_log_print(ANDROID_LOG_VERBOSE, "C++ Tracking", "Matrix %s############################", text.c_str());
    __android_log_print(ANDROID_LOG_VERBOSE, "C++ Tracking", "%s [%f %f %f]", text.c_str() , mat.at<float>(0,0),mat.at<float>(0,1),mat.at<float>(0,2));
    __android_log_print(ANDROID_LOG_VERBOSE, "C++ Tracking", "%s [%f %f %f]", text.c_str() , mat.at<float>(1,0),mat.at<float>(1,1),mat.at<float>(1,2));
    __android_log_print(ANDROID_LOG_VERBOSE, "C++ Tracking", "%s [%f %f %f]", text.c_str() , mat.at<float>(2,0),mat.at<float>(2,1),mat.at<float>(2,2));
    __android_log_print(ANDROID_LOG_VERBOSE, "C++ Tracking", "Matrix ##############################");
}

Rect findResultROI(vector<Rect> rects){
    if(rects[0].width == 0 && rects[0].height == 0){
        return rects[1];
    }
    Point tl(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    Point br(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
    for(int i = 0 ; i < rects.size();i++){
        tl.x = std::min(tl.x, rects[i].x);
        tl.y = std::min(tl.y, rects[i].y);
        br.x = std::max(br.x, rects[i].x + rects[i].width);
        br.y = std::max(br.y, rects[i].y + rects[i].height);
    }
    Rect r(tl,br);
    return r;
}
Rect findROI(float warped_image_scale, Mat image ,CameraParams param ){
    // 3.0 = 1920 * 1080
    double compose_work_aspect = 3.0;
    clock_t c_t0 = std::clock();
    float u,v,w;
    int count = 0;
    float vec[3] = {0,0,1};
    Mat R,K,K_inv;
    float r_kinv[9] = {0};
    float out[9];
    printMatrix(param.R,"ROI Camera");
    float tl_x = MAXFLOAT;
    float tl_y = MAXFLOAT;
    float br_x = -MAXFLOAT;
    float br_y = -MAXFLOAT;
    param.R.convertTo(R,CV_32F);
    param.focal *= compose_work_aspect;
    param.ppx *= compose_work_aspect;
    param.ppy *= compose_work_aspect;
    __android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","Image type : %d ",image.type());

    K = param.K().inv();
    int counter = 0;
    K.convertTo(K_inv,CV_32F);
    Mat r_kinv_mat = R * K_inv;
    //K == double ? R == float?
    for(int q = 0 ; q < 3 ; q++){
        for(int s = 0; s < 3 ; s++){
            r_kinv[q*3+s] = r_kinv_mat.at<float>(q,s);
        }
    }
    __android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI"," Size : %d %d [%d]",image.cols,image.rows,image.total());
    for(int x = 0 ; x < image.rows * 3;x++) {
        //left
        vec[0] = 0;
        vec[1] = x;

        vectorMatrixMultiply(vec, r_kinv, out);
        u = warped_image_scale * compose_work_aspect * atan2f(out[0], out[2]);
        w = out[1] / sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
        v = warped_image_scale * compose_work_aspect *
            (static_cast<float>(CV_PI) - acosf(w == w ? w : 0));
        if (u < tl_x) tl_x = u;
        if (v < tl_y) tl_y = v;
        if (u > br_x) br_x = u;
        if (v > br_y) br_y = v;
        counter++;
    }
    __android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","1 sizey %d",image.rows);
    for(int x = 0 ; x < image.rows * 3 ;x++) {
        //right
        vec[0] = image.cols-1;
        vec[1] = x;
        vectorMatrixMultiply(vec,r_kinv,out);
        u = warped_image_scale*compose_work_aspect * atan2f(out[0], out[2]);
        w = out[1] / sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
        v = warped_image_scale*compose_work_aspect * (static_cast<float>(CV_PI) - acosf(w == w ? w : 0));

        if(u < tl_x) tl_x = u;
        if(v < tl_y) tl_y = v;
        if(u > br_x) br_x = u;
        if(v > br_y) br_y = v;
        counter++;
    }

    __android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","2 , size %d",image.total());
    for(int y = 0 ; y< image.cols * 3 ;y++) {
        //top
        vec[0] = y;
        vec[1] = 0;

        vectorMatrixMultiply(vec, r_kinv, out);
        u = warped_image_scale * compose_work_aspect * atan2f(out[0], out[2]);
        w = out[1] / sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
        v = warped_image_scale * compose_work_aspect *
            (static_cast<float>(CV_PI) - acosf(w == w ? w : 0));

        if (u < tl_x) tl_x = u;
        if (v < tl_y) tl_y = v;
        if (u > br_x) br_x = u;
        if (v > br_y) br_y = v;
        counter++;
    }

    __android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","3 , sizey %d",image.cols);
    for(int y = 0 ; y< image.cols * 3 ;y++) {
        //bottom
        vec[0] = y;
        vec[1] = image.rows-1;
        vectorMatrixMultiply(vec,r_kinv,out);
        u = warped_image_scale*compose_work_aspect * atan2f(out[0], out[2]);
        w = out[1] / sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
        v = warped_image_scale*compose_work_aspect * (static_cast<float>(CV_PI) - acosf(w == w ? w : 0));

        if(u < tl_x) tl_x = u;
        if(v < tl_y) tl_y = v;
        if(u > br_x) br_x = u;
        if(v > br_y) br_y = v;
        counter++;
    }

    __android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","4");

    __android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","%f %f %f %f",tl_x,tl_y,br_x,br_y);

    clock_t c_t1 = std::clock();
    __android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI"," %lf %d",(double)(c_t1-c_t0)/CLOCKS_PER_SEC,count);

    return Rect(Point(tl_x,tl_y),Point(br_x,br_y));
}
extern "C" {
vector<Point2f> trackedFeatures;
bool freshStart;
bool setup = true;
Rect last_rect;
CameraParams last_param;
Mat last_image;

Ptr<FeatureDetector> tracking_dectector = GFTTDetector::create(300,0.01,10,3);
    JNIEXPORT int JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_track(JNIEnv *env, jobject instance, jlong imgAddr, jlong retAddr, jlong areaAddr, jlong rotAddr, jlong refindRotAddr, jlong roiAddr, jlong k_rinvAddr) {

        if(setup){
            last_param.ppx = 497.82610894280418/3.0;
            last_param.ppy = 923.84763460447925/3.0;
            last_param.focal = 1469.7969239257427/3.0;
            last_param.aspect = 1.0;
            setup = false;
        }

        Mat_<float> rigidTransform;
        Mat& full_img = *(Mat*)imgAddr;
        Mat& rot = *(Mat*)rotAddr;
        Mat& refinedRot = *(Mat*)refindRotAddr;
        Mat& roiRot = *(Mat*)roiAddr;
        Mat& k_rinv = *(Mat*)k_rinvAddr;
        Mat& area = *(Mat*)areaAddr;
        clock_t c_m1 = clock();
        Mat img;
        resize(full_img,img,Size(640,360));
        flip(img.t(),img,0);
        Mat gray; cvtColor(img,gray,CV_BGR2GRAY);
        vector<KeyPoint> corners;
        clock_t c_m15 = clock();
        if(trackedFeatures.size() < 100) {
            clock_t c_m2 = clock();
            //FAST(gray,corners,1, true,FastFeatureDetector::TYPE_9_16);

            tracking_dectector->detect(gray,corners);
            //goodFeaturesToTrack(gray,corners,300,0.01,10);
            clock_t c_m3 = clock();
            for (int i = 0; i < corners.size() ; ++i) {
                trackedFeatures.push_back(corners[i].pt);

            }
            __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","[Time] Found : %d Feature in %lf sec.",corners.size(),double(c_m3-c_m2)/CLOCKS_PER_SEC);
            if(!last_image.empty()){
                //do update prevCamera.R
                //last_param.R = rot;
            }
        }

        if(!last_image.empty()) {
            vector<Point2f> valid_point;
            vector<uchar> status; vector<float> errors;
            clock_t c_m4 = clock();
            imwrite("/sdcard/stitch/last.jpg",last_image);
            imwrite("/sdcard/stitch/gray.jpg", gray);
            calcOpticalFlowPyrLK(last_image,gray,trackedFeatures,valid_point,status,errors,Size(10,10));
            clock_t c_m5 = clock();
            int valid_feature = countNonZero(status);
            __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Valid Feature %d from %d %d ",valid_feature,status.size(),valid_point.size());
            if( valid_feature < status.size() * 0.8) {
                //prevCamera.R = Mat::eye(3,3,CV_32FC1);
                trackedFeatures.clear();
                last_image.release();
                __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Fresh Start");
                freshStart = true;
                return 0;
            } else {
                freshStart = false;
            }
            vector<Point2f> original_valid_points;
            vector<Point2f> updated_valid_points;
            vector<Point2f> good_original_valid_points;
            vector<Point2f> good_updated_valid_points;
            for (int i = 0; i < status.size(); ++i) {
                if(status[i]) {
                    original_valid_points.push_back(trackedFeatures[i]);
                    updated_valid_points.push_back(valid_point[i]);
                }
            }
            clock_t c_m6 = clock();
            vector<CameraParams> camera_set(2);
            CameraParams currentCamera;
            if(freshStart){
                last_param.R = Mat::eye(3,3,CV_32F);
                freshStart = false;
            }
            camera_set[0] = last_param;

            currentCamera.aspect = camera_set[0].aspect;
            currentCamera.focal = camera_set[0].focal;
            currentCamera.ppx = camera_set[0].ppx;
            currentCamera.ppy = camera_set[0].ppy;
            currentCamera.R = camera_set[0].R;

            camera_set[1] = currentCamera;
            vector<uchar> good_points;

            estimateRigidTransform(original_valid_points,updated_valid_points, false, good_points);
            int good_point_count = 0;
            for(int i = 0 ; i < good_points.size() ; i++){
                if(good_points[i]){
                    good_point_count+=1;
                    good_original_valid_points.push_back(original_valid_points[i]);
                    good_updated_valid_points.push_back(updated_valid_points[i]);
                }
            }
            __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Good Point %d from %d %d %d",good_point_count,good_points.size(),original_valid_points.size(),updated_valid_points.size());
//            Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures,valid_point,false);
            int iterationCount = minimizeRotation(good_original_valid_points, good_updated_valid_points, camera_set, 1);
            clock_t c_m7 = clock();
            last_param = camera_set[1];

            trackedFeatures.clear();
            trackedFeatures = updated_valid_points;
            __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Iteration : %d using %d points",iterationCount,good_updated_valid_points.size());
            __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","[Time] Calc optflow %lf, est transform %lf",double(c_m5-c_m4)/CLOCKS_PER_SEC,double(c_m7-c_m6)/CLOCKS_PER_SEC);

            printMatrix(rot,"original Rotation");
            printMatrix(camera_set[1].R,"updated Rotation");

            vector<double> focals;
            for (size_t i = 0; i < camera_set.size(); ++i) {
                focals.push_back(camera_set[i].focal);
            }

            sort(focals.begin(), focals.end());
            float warped_image_scale;
            if (focals.size() % 2 == 1)
                warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
            else
                warped_image_scale =
                        static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) *
                        0.5f;
            Rect current_rect = findROI(warped_image_scale, gray, camera_set[1]);
            __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","C.Rect tl(%d:%d) size[%d:%d]",current_rect.x,current_rect.y,current_rect.width,current_rect.height);

            vector<Rect> rects(2);
            rects[0] = last_rect;
            rects[1] = current_rect;

            __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Rect tl x:y (%d:%d) width:height[%d:%d]",last_rect.x,last_rect.y,last_rect.width,last_rect.height);

            Mat rinv_temp = camera_set[1].R.inv();
//            Mat rinv_temp = Mat::eye(3,3,CV_32F).inv();
            Mat k_temp;
            Mat rinv32_temp;
            rinv_temp.convertTo(rinv32_temp,CV_32F);
            CameraParams realCamera;
            realCamera.aspect = currentCamera.aspect;
            realCamera.focal = currentCamera.focal * 3;
            realCamera.ppx = currentCamera.ppx * 3;
            realCamera.ppy = currentCamera.ppy * 3;
            realCamera.K().convertTo(k_temp, CV_32F);
            k_rinv = k_temp * rinv32_temp;
            printMatrix(k_rinv,"K_Rinv");
            printMatrix(k_temp,"K_Rinv2");
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI 0 : %d %d %d %d", rects[0].x,rects[0].y, rects[0].width, rects[0].height);
            int work_width = (warped_image_scale) * M_PI * 2;
            roiRot.at<int>(0, 0) = last_rect.x;
            roiRot.at<int>(0, 1) = last_rect.y;
            roiRot.at<int>(0, 2) = last_rect.width;
            roiRot.at<int>(0, 3) = last_rect.height;

            roiRot.at<int>(1, 0) = current_rect.x;
            roiRot.at<int>(1, 1) = current_rect.y;
            roiRot.at<int>(1, 2) = current_rect.width;
            roiRot.at<int>(1, 3) = current_rect.height;
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI 1 : %d %d %d %d", rects[1].x,rects[1].y, rects[1].width, rects[1].height);



            for(int i = 0; i < 4 ; i++){
                for(int j = 0; j < 4 ;j++){
                    if(i == 3 && j == 3){
                        refinedRot.at<float>(i,j) = 1;
                    }
                    else if(i == 3 || j == 3){
                        refinedRot.at<float>(i,j) = 0;
                    }
                    else{
                        refinedRot.at<float>(i,j) = currentCamera.R.at<float>(i,j);
                    }

                }
            }
            refinedRot = Mat::eye(4,4,CV_32F);



            last_rect = findResultROI(rects);
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI R : %d %d %d %d", last_rect.x,last_rect.y, last_rect.width, last_rect.height);

            //* 3.0 for 1920*1080 (640*360 input)
            area.at<float>(0, 0) = (work_width * 3.0 / 2) + last_rect.x;
            area.at<float>(0, 1) = last_rect.y;
            area.at<float>(0, 2) = last_rect.width;
            area.at<float>(0, 3) = last_rect.height;
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI A : %f %f %f %f", area.at<float>(0, 0),area.at<float>(0, 1), area.at<float>(0, 2), area.at<float>(0, 3));


        }
        else{
            gray.copyTo(last_image);
            return -1;
        }
        clock_t c_m8 = clock();
        gray.copyTo(last_image);

        clock_t c_m9 = clock();
        __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","[Time] cvt to gray %lf copy prevframe %lf",double(c_m15-c_m1)/CLOCKS_PER_SEC,double(c_m9-c_m8)/CLOCKS_PER_SEC);


        __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","[Time] Total : %lf",double(c_m9-c_m1)/CLOCKS_PER_SEC);
        return 1;
    }

}
#endif