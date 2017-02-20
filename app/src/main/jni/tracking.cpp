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
#include "BundleCV.h"

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
float calcDistance(Point2f p1,Point2f p2){
    return sqrt(((p2.x-p1.x)*(p2.x-p1.x))+((p2.y-p1.y)*(p2.y-p1.y)));
}
void findInliner(vector<Point2f> points1,vector<Point2f> points2,vector<uchar> &status){
    //sort & find median
    float range = 20.0;
    int count = 0;
    vector<float> distances_x,distances_x_clone;
    vector<float> distances_y,distances_y_clone;
    for(int i = 0 ; i < points1.size() ;i++){
        //__android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Inliner Point p1 : p2 (x,y) ; (%f,%f) (%f,%f)",points1[i].x,points1[i].y,points2[i].x,points2[i].y);
        distances_x.push_back(points1[i].x-points2[i].x);
        distances_x_clone.push_back(distances_x[i]);
        distances_y.push_back(points1[i].y-points2[i].y);
        distances_y_clone.push_back(distances_y[i]);

        __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Inliner distance (x,y) : (%f,%f)",distances_x[i],distances_y[i]);
    }
    sort(distances_x_clone.begin(),distances_x_clone.end());
    sort(distances_y_clone.begin(),distances_y_clone.end());
    float median_x = distances_x_clone[distances_x_clone.size()/2];
    float median_y = distances_y_clone[distances_y_clone.size()/2];
    //get all point along that median
    __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Inliner Median (x,y) : (%f,%f)",median_x,median_y);
    for(int i = 0; i < points1.size() ;i++){
        if(distances_x[i] > median_x - range && distances_x[i] < median_x + range && distances_y[i] > median_y - range && distances_y[i] < median_y + range){
            status[i] = 1;
        }
        else{
            status[i] = 0;
            count+=1;
        }
    }
    __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Inliner != Count %d",count);

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
Rect findROI(float warped_image_scale, Mat image ,CameraParams param, float processing_ratio ){
    // 3.0 = 1920 * 1080
    double compose_work_aspect = processing_ratio;
    clock_t c_t0 = std::clock();
    float u,v,w;
    int count = 0;
    float vec[3] = {0,0,1};
    Mat R,K,K_inv;
    float r_kinv[9] = {0};
    float out[9];
    //printMatrix(param.R,"ROI Camera");
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
    for(int x = 0 ; x < image.rows * processing_ratio;x++) {
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
    for(int x = 0 ; x < image.rows * processing_ratio ;x++) {
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
    for(int y = 0 ; y< image.cols * processing_ratio ;y++) {
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
    for(int y = 0 ; y< image.cols * processing_ratio ;y++) {
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
bool firstImage = true;
CameraParams main_param;
vector<uchar> tracking_points;
int image_size;
Mat last_image;
Mat main_image;
Rect last_rect;
int max_feature = 300;
float processing_ratio = 1.0;
vector<Point2f> main_points;
bool stop = false;

Ptr<FeatureDetector> tracking_dectector = GFTTDetector::create(max_feature,0.01,10,3);
JNIEXPORT int JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_track(JNIEnv *env, jobject instance, jlong imgAddr, jlong retAddr, jlong areaAddr, jlong rotAddr, jlong refindRotAddr, jlong roiAddr, jlong k_rinvAddr) {
    if(stop) {
        __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","STOP");
        return 1;
    }
    Mat& full_img = *(Mat*)imgAddr;
    Mat& rot = *(Mat*)rotAddr;
    Mat& refinedRot = *(Mat*)refindRotAddr;
    Mat& roiRot = *(Mat*)roiAddr;
    Mat& k_rinv = *(Mat*)k_rinvAddr;
    Mat& area = *(Mat*)areaAddr;
    clock_t c_m1 = clock();
    Mat img;
    resize(full_img,img,Size(1920.0/processing_ratio,1080.0/processing_ratio));
    flip(img.t(),img,1);
    Mat gray; cvtColor(img,gray,CV_BGR2GRAY);
    vector<KeyPoint> corners;
    clock_t c_m15 = clock();
    //First Frame
    if(firstImage){

        matcher::create(0.3f, 6, 6);
        tracking_dectector->detect(gray,corners);

        for (int i = 0; i < corners.size() ; ++i) {
            trackedFeatures.push_back(corners[i].pt);
            tracking_points.push_back(i);
        }
        main_points = trackedFeatures;

        CameraParams currentCamera;
        currentCamera.aspect = 1.0;
        currentCamera.focal = 1453.8/processing_ratio;
        currentCamera.ppx = 540/processing_ratio;
        currentCamera.ppy = 960/processing_ratio;
        currentCamera.R = Mat::eye(3,3,CV_32F);
        CameraParams realCamera;
        realCamera.aspect = currentCamera.aspect;
        realCamera.focal = currentCamera.focal * processing_ratio;
        realCamera.ppx = currentCamera.ppx * processing_ratio;
        realCamera.ppy = currentCamera.ppy * processing_ratio;

        float warped_image_scale = currentCamera.focal;
        Rect current_rect = findROI(warped_image_scale, gray, currentCamera, processing_ratio);

        __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking",
                            "C.Rect tl(%d:%d) size[%d:%d]", current_rect.x, current_rect.y,
                            current_rect.width, current_rect.height);


        Mat rinv_temp = currentCamera.R.inv();
        Mat k_temp;
        Mat rinv32_temp;
        rinv_temp.convertTo(rinv32_temp, CV_32F);

        realCamera.K().convertTo(k_temp, CV_32F);
        k_rinv = k_temp * rinv32_temp;

        int work_width = (warped_image_scale) * M_PI * 2;
        roiRot.at<int>(0, 0) = 0;
        roiRot.at<int>(0, 1) = 0;
        roiRot.at<int>(0, 2) = 0;
        roiRot.at<int>(0, 3) = 0;

        roiRot.at<int>(1, 0) = current_rect.x;
        roiRot.at<int>(1, 1) = current_rect.y;
        roiRot.at<int>(1, 2) = current_rect.width;
        roiRot.at<int>(1, 3) = current_rect.height;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI 1 : %d %d %d %d",
                            current_rect.x, current_rect.y, current_rect.width, current_rect.height);


        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                if (i == 3 && j == 3) {
                    refinedRot.at<float>(i, j) = 1;
                }
                else if (i == 3 || j == 3) {
                    refinedRot.at<float>(i, j) = 0;
                }
                else {
                    refinedRot.at<float>(i, j) = currentCamera.R.at<float>(i, j);
                }

            }
        }
        refinedRot = Mat::eye(4, 4, CV_32F);


        __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI R : %d %d %d %d",
                            current_rect.x, current_rect.y, current_rect.width, current_rect.height);

        //* 3.0 for 1920*1080 (640*360 input)
        area.at<float>(0, 0) = (work_width * processing_ratio / 2) + current_rect.x;
        area.at<float>(0, 1) = current_rect.y;
        area.at<float>(0, 2) = current_rect.width;
        area.at<float>(0, 3) = current_rect.height;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI A : %f %f %f %f",
                            area.at<float>(0, 0), area.at<float>(0, 1),
                            area.at<float>(0, 2), area.at<float>(0, 3));
        last_rect = current_rect;
        main_param = currentCamera;
        firstImage = false;
        gray.copyTo(last_image);
        gray.copyTo(main_image);
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "Upload First Image");
        image_size++;
        return 2;

    }


        //Optical flow on all frames
    else {
        vector<Point2f> valid_point;
        vector<uchar> status; vector<float> errors;
        clock_t c_m4 = clock();
        calcOpticalFlowPyrLK(last_image,gray,trackedFeatures,valid_point,status,errors,Size(10,10));
        clock_t c_m5 = clock();
        int valid_feature = countNonZero(status);
        __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Valid Feature %d from %d %d ",valid_feature,status.size(),valid_point.size());

        vector<Point2f> original_valid_points;
        vector<Point2f> updated_valid_points;
        vector<uchar> update_tracking_points;
        for (int i = 0; i < status.size(); ++i) {
            if(status[i]) {
                original_valid_points.push_back(trackedFeatures[i]);
                updated_valid_points.push_back(valid_point[i]);
                update_tracking_points.push_back(tracking_points[i]);
            }
        }


        trackedFeatures = updated_valid_points;
        tracking_points = update_tracking_points;
        if(valid_feature < max_feature * 0.9) {

            vector<Point2f> good_original_valid_points;
            vector<Point2f> good_updated_valid_points;
            vector<Point2f> main_comparison_points;
            vector<uchar> good_tracking_points;
            vector<uchar> good_tracking_points_updated;

            ImageFeatures if_src;
            ImageFeatures if_dst;
            clock_t c_m6 = clock();
            vector<CameraParams> camera_set(2);
            CameraParams currentCamera;

            camera_set[0] = main_param;
            currentCamera.aspect = camera_set[0].aspect;
            currentCamera.focal = camera_set[0].focal;
            currentCamera.ppx = camera_set[0].ppx;
            currentCamera.ppy = camera_set[0].ppy;
            currentCamera.R = camera_set[0].R;

            camera_set[1] = currentCamera;
            vector<uchar> good_points(original_valid_points.size());
            int good_point_count = 0;
            int bad_point_count = 0;
            //FORCE NO RANSAC
            vector<Point2f> main_comparison_points_raw;
            for(int i = 0 ; i < tracking_points.size();i++){
                main_comparison_points_raw.push_back(main_points[tracking_points[i]]);
            }

            Mat H = matcher::findHomography(main_comparison_points_raw, updated_valid_points, CV_RANSAC, 5.0, good_points);


            vector<KeyPoint> main_comparison_kp;
            vector<KeyPoint> good_updated_valid_kp;
            vector<DMatch> matches_pair;
            for (int i = 0; i < good_points.size(); i++) {
                if (good_points[i]) {
                    good_original_valid_points.push_back(original_valid_points[i]);
                    good_updated_valid_points.push_back(updated_valid_points[i]);
                    good_tracking_points.push_back(tracking_points[i]);
                    main_comparison_points.push_back(main_points[tracking_points[i]]);
                    //for drawing
                        KeyPoint kp1(main_comparison_points[good_point_count], 1.0);
                        KeyPoint kp2(good_updated_valid_points[good_point_count], 1.0);
                        main_comparison_kp.push_back(kp1);
                        good_updated_valid_kp.push_back(kp2);
                        DMatch dm(good_point_count, good_point_count, 1.0);
                        matches_pair.push_back(dm);
                        good_tracking_points_updated.push_back(1);
                        good_point_count += 1;
                    //end of drawing
                }
            }

            printMatrix(camera_set[0].R,"Last Camera");
            printMatrix(camera_set[1].R,"This Camera");
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking",
                                "Good Point %d from %d %d %d", good_point_count,
                                good_points.size(), original_valid_points.size(),
                                updated_valid_points.size());
            Mat good_out;

            drawMatches(main_image,main_comparison_kp,gray,good_updated_valid_kp,matches_pair,good_out);

//            char kp_filename[50];
//            char kp_filename2[50];
//            sprintf(kp_filename,"/sdcard/stitch/main%d.jpg",image_size);
//            sprintf(kp_filename2,"/sdcard/stitch/last%d.jpg",image_size);
//            imwrite(kp_filename,main_image);
//            imwrite(kp_filename2,gray);



//            char pair_filename[50];
//            sprintf(pair_filename,"/sdcard/stitch/pair_good%d.jpg",image_size);
//            imwrite(pair_filename,good_out);

            //something wrong here
//            int iterationCount = minimizeRotation(main_comparison_points, good_updated_valid_points, camera_set, 1);
            camera_set[1].R = rot;
            estimate_(main_comparison_points,good_updated_valid_points, main_image.size(),camera_set);
//            (*adjuster)(nearest_feature,pairwise_matches , camera_set);
            printMatrix(camera_set[0].K(),"Camera K_1");
            printMatrix(camera_set[1].K(),"Camera K_2");
            printMatrix(camera_set[0].R,"Camera R_1");
            printMatrix(camera_set[1].R,"Camera R_2");
            clock_t c_m7 = clock();
            currentCamera = camera_set[1];

            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking",
                                "[Time] Calc optflow %lf, est transform %lf",
                                double(c_m5 - c_m4) / CLOCKS_PER_SEC,
                                double(c_m7 - c_m6) / CLOCKS_PER_SEC);
            //camera_set[1].R = rot;
            printMatrix(camera_set[1].R,"Minimized Rotation");
            printMatrix(rot,"Gyro Rotation");
            printMatrix(camera_set[1].K(),"K");
            trackedFeatures.clear();
            trackedFeatures = updated_valid_points;

            //printMatrix(rot, "original Rotation");
            //printMatrix(camera_set[1].R, "updated Rotation");

            float warped_image_scale = 1453.8f;
            Rect current_rect = findROI(warped_image_scale, gray, camera_set[1], processing_ratio);

            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking",
                                "C.Rect tl(%d:%d) size[%d:%d]", current_rect.x, current_rect.y,
                                current_rect.width, current_rect.height);

            vector<Rect> rects(2);
            rects[0] = last_rect;
            rects[1] = current_rect;

            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking",
                                "Rect tl x:y (%d:%d) width:height[%d:%d]", last_rect.x,
                                last_rect.y, last_rect.width, last_rect.height);

            Mat rinv_temp = camera_set[1].R.inv();
            Mat k_temp;
            Mat rinv32_temp;
            rinv_temp.convertTo(rinv32_temp, CV_32F);
            CameraParams realCamera;
            realCamera.aspect = currentCamera.aspect;
            realCamera.focal = currentCamera.focal * processing_ratio;
            realCamera.ppx = currentCamera.ppx * processing_ratio;
            realCamera.ppy = currentCamera.ppy * processing_ratio;
            realCamera.K().convertTo(k_temp, CV_32F);
            k_rinv = k_temp * rinv32_temp;

            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI 0 : %d %d %d %d",
                                rects[0].x, rects[0].y, rects[0].width, rects[0].height);
            int work_width = (warped_image_scale) * M_PI * 2;
            roiRot.at<int>(0, 0) = last_rect.x;
            roiRot.at<int>(0, 1) = last_rect.y;
            roiRot.at<int>(0, 2) = last_rect.width;
            roiRot.at<int>(0, 3) = last_rect.height;

            roiRot.at<int>(1, 0) = current_rect.x;
            roiRot.at<int>(1, 1) = current_rect.y;
            roiRot.at<int>(1, 2) = current_rect.width;
            roiRot.at<int>(1, 3) = current_rect.height;
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI 1 : %d %d %d %d",
                                rects[1].x, rects[1].y, rects[1].width, rects[1].height);
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking","Work-Width : %d",work_width);

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    if (i == 3 && j == 3) {
                        refinedRot.at<float>(i, j) = 1;
                    }
                    else if (i == 3 || j == 3) {
                        refinedRot.at<float>(i, j) = 0;
                    }
                    else {
                        refinedRot.at<float>(i, j) = camera_set[1].R.at<float>(i, j);
                    }

                }
            }
            refinedRot = Mat::eye(4, 4, CV_32F);


            last_rect = findResultROI(rects);
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI R : %d %d %d %d",
                                last_rect.x, last_rect.y, last_rect.width, last_rect.height);

            //* 3.0 for 1920*1080 (640*360 input)

            area.at<float>(0, 0) = (work_width * processing_ratio / 2) + last_rect.x;
            area.at<float>(0, 1) = last_rect.y;
            area.at<float>(0, 2) = last_rect.width;
            area.at<float>(0, 3) = last_rect.height;
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "ROI A : %f %f %f %f",
                                area.at<float>(0, 0), area.at<float>(0, 1),
                                area.at<float>(0, 2), area.at<float>(0, 3));
            trackedFeatures.clear();
            tracking_points.clear();
            tracking_dectector->detect(gray,corners);

            for (int i = 0; i < corners.size() ; ++i) {
                trackedFeatures.push_back(corners[i].pt);
                tracking_points.push_back(i);
            }
            main_points = trackedFeatures;
            main_param.ppx = currentCamera.ppx;
            main_param.ppy = currentCamera.ppy;
            main_param.aspect = currentCamera.aspect;
            main_param.focal = currentCamera.focal;
            main_param.R = currentCamera.R;
            printMatrix(main_param.R,"Current R");
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "Current focal %lf",currentCamera.focal);
            __android_log_print(ANDROID_LOG_DEBUG, "C++ Tracking", "Upload Image");
            gray.copyTo(last_image);
            gray.copyTo(main_image);
            image_size++;
//            stop = true;
            return 2;

        }
        else{
            gray.copyTo(last_image);
            return 1;
        }

    }

}

}
#endif