

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//
//M*/
#pragma ide diagnostic ignored "ArrayIssue"
#define EXPERIMENT 0
#include "stitching.h"

using namespace std;
using namespace cv;
using namespace cv::detail;

void findDescriptor(Mat img,std::vector<KeyPoint> &keypoints ,Mat &descriptor){
	if(detector_setup){
		//for surf
//		detector->set("hessianThreshold", 300);
//		detector->set("nOctaves", 3);
//		detector->set("nOctaveLayers", 4);
		detector_setup = 0;
		detector->set("nFeatures", 100);
	}
	Mat gray_img;
    Mat mask;
    mask.create(img.size(), CV_8U);
    mask.setTo(Scalar::all(255));
    int x_margin = img.rows/3;
    int y_margin = img.cols/3;
//    for(int i = x_margin ; i < img.rows - x_margin;i++){
//        for(int j = y_margin ; j < img.cols - y_margin;j++){
//            mask.at<uchar>(i,j) = 0;
//        }
//    }
    cvtColor(img,gray_img,CV_BGR2GRAY);
	(*detector)(gray_img , mask, keypoints, descriptor, false);
	descriptor = descriptor.reshape(1, static_cast<int>(keypoints.size()));
    Mat out;
    drawKeypoints(img,keypoints,out);
    char file_str[50]; // enough to hold all numbers up to 64-bits
    sprintf(file_str, "/sdcard/stitch/keypoint%d.jpg", images.size());
    imwrite(file_str,out);
}

inline float calcDistance(float x1,float y1,float z1,float x2,float y2, float z2){
	return ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2))+((z1-z2)*(z1-z2));
}

int findNearest(int from, int to, std::vector<ImagePackage> images,Mat &inputR){
	int nearest_index = -1;
	CameraParams dummy_K = images[0].param;
	dummy_K.ppx = images[0].feature.img_size.width/2;
	dummy_K.ppy = images[0].feature.img_size.height/2;
	dummy_K.focal = (images[0].feature.img_size.width * 4.7 / focal_divider);
	size_t y;
	Mat dummy_K_mat;
	dummy_K.K().convertTo(dummy_K_mat,CV_32F);
	float max_distance = FLT_MAX;
	for(int i = from ; i < to; i++){
		Mat test_point(1,3,CV_32F,Scalar(1));
		Mat out = test_point * inputR;
		Mat out2 = test_point * images[i].param.R;
		float dist = calcDistance(out2.at<float>(0,0),out2.at<float>(0,1),out2.at<float>(0,2),out.at<float>(0,0),out.at<float>(0,1),out.at<float>(0,2));
		if(dist < max_distance){
			max_distance = dist;
			nearest_index = i;
		}
	}
	return nearest_index;
}


JNIEXPORT void JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_nativeAligning(JNIEnv*, jobject, jlong imgaddr,jlong glrotaddr,jlong retaddr){
	__android_log_print(ANDROID_LOG_DEBUG,"C++ aligning","Start");

	clock_t c_start = std::clock();
	Mat& full_img  = *(Mat*)imgaddr;
	transpose(full_img, full_img);
    flip(full_img, full_img,0);
	Mat img;
	resize(full_img,img,Size(),work_scale,work_scale);

	Mat& gl_rot = *(Mat*) glrotaddr;
	ImageFeatures input_feature;
	Mat input_descriptor;
	//imwrite("/sdcard/stitch/tracking2.jpg",full_img);
	clock_t c_before_desc = std::clock();
	findDescriptor(img, input_feature.keypoints, input_feature.descriptors);
	clock_t c_after_desc = std::clock();
	vector<MatchesInfo> tracking_matches;
	//const descriptor (img1)
	vector<ImageFeatures> tracking_feature(2);
	input_feature.img_idx = -1;
	input_feature.img_size = img.size();
	Mat input_R(3,3,CV_32F);
	for(int j = 0 ; j < 3 ;j++){
		for(int k = 0; k < 3 ;k++){
			input_R.at<float>(j,k) = gl_rot.at<float>(j,k);
		}
	}
	int nearest_index = findNearest(0,images.size(),images,input_R);
    //imwrite("/sdcard/stitch/tracking3.jpg",images[nearest_index].full_img);
	tracking_feature[0] = images[nearest_index].feature;
	tracking_feature[1] = input_feature;
	clock_t c_before_matcher = std::clock();
	matcher::match(tracking_feature,tracking_matches);
	clock_t c_after_matcher = std::clock();
	vector<CameraParams> tracking_cameras(2);
	int matches_index = - 1;

	for(int i = 0 ; i < tracking_matches.size() ;i++){
		if(tracking_matches[i].dst_img_idx == 1 && tracking_matches[i].src_img_idx == 0){
			__android_log_print(ANDROID_LOG_DEBUG,"C++ aligning","Used Pair ,%d %d %d",tracking_matches[i].src_img_idx , tracking_matches[i].dst_img_idx,tracking_matches[i].matches.size());
			matches_index = i;
			break;
		}
	}

	for(int i = 0 ; i < 2; i++){
		CameraParams tracking_camera;
		tracking_cameras[i].ppy = images[nearest_index].param.ppy;
		tracking_cameras[i].ppx = images[nearest_index].param.ppx;
		tracking_cameras[i].focal = images[nearest_index].param.focal;
		tracking_cameras[i].aspect = 1;
		if( i == 1 ){
			tracking_camera.R = input_R;
		}
		else{
			tracking_camera.R = images[nearest_index].param.R;
		}
		tracking_cameras[i] = tracking_camera;

	}
	vector<Point2f> src_points;
	vector<Point2f> dst_points;
	for(int i = 0 ; i < tracking_matches[matches_index].matches.size();i++){
		if(tracking_matches[matches_index].inliers_mask[i]){
			src_points.push_back(tracking_feature[0].keypoints[tracking_matches[matches_index].matches[i].queryIdx].pt);
			dst_points.push_back(tracking_feature[1].keypoints[tracking_matches[matches_index].matches[i].trainIdx].pt);
		}
	}
	clock_t c_before_bundle = clock();
	minimizeRotation(src_points,dst_points,tracking_cameras);
	clock_t c_after_bundle = clock();
	Mat& R = *(Mat*)retaddr;
	for(int i = 0; i < 4 ; i++){
		for(int j = 0; j < 4 ;j++){
			if(i == 3 && j == 3){
				R.at<float>(i,j) = 1;
			}
			else if(i == 3 || j == 3){
				R.at<float>(i,j) = 0;
			}
			else{
				R.at<float>(i,j) = tracking_cameras[1].R.at<float>(i,j);
			}

		}
	}
	clock_t c_end = clock();
	double a = ((c_end-c_start)/(double)CLOCKS_PER_SEC);
	double b = ((c_after_bundle-c_before_bundle)/(double)CLOCKS_PER_SEC);
	double c = ((c_after_matcher-c_before_matcher)/(double)CLOCKS_PER_SEC);
	double d = ((c_after_desc-c_before_desc)/(double)CLOCKS_PER_SEC);
	double e = ((c_before_desc-c_start)/(double)CLOCKS_PER_SEC);


	__android_log_print(ANDROID_LOG_DEBUG,"C++ aligning"," Timer [ All : %lf Prepare : %lf Desc : %lf Matcher : %lf  Bundle : %lf ]",a,e,d,c,b);

//
// 	Mat K;
//	tracking_cameras[1].K().convertTo(K,CV_32F);

// 	H = K * tracking_cameras[1].R * K.inv();
	//return only R no-need for homography
//	H = tracking_cameras[1].R;


//
//	vector<Point2f> in_point;
//	vector<Point2f> in2_point;
//	__android_log_print(ANDROID_LOG_DEBUG,"MatchCount","%d",matches.size());
//	int viewport[4] = {0,0,GL_WIDTH,GL_HEIGHT};
//	Mat& gl_proj = *(Mat*) glprojaddr;
//
//
//	for(int i = 0 ; i < matches.size() ;i++){
//		float screenCoord[3];
//		Point2f xy1 = input_keypoint[matches[i].queryIdx].pt;
//		xy1.x /= tracking_scale;
//		xy1.y /= tracking_scale;
//		//const descriptor (img1)
//		Point2f xy2 = p2d[1][matches[i].trainIdx];
//
//
//		__android_log_print(ANDROID_LOG_DEBUG,"MatchPoint2D","(%f,%f) (%f,%f)",xy1.x,xy1.y,xy2.x,xy2.y);
//		Point3f xyz2 = p3d[1][matches[i].trainIdx];
//		__android_log_print(ANDROID_LOG_DEBUG,"MatchPoint3DRaw","%f %f %f",xyz2.x,xyz2.y,xyz2.z);
//		glhProjectf(xyz2.x,xyz2.y,xyz2.z,(float*)gl_rot.data,(float*)gl_proj.data,viewport,screenCoord);
//		if(screenCoord[0] > 0 && screenCoord[0] < GL_WIDTH && screenCoord[1] > 0 && screenCoord[1] < GL_HEIGHT){
//			__android_log_print(ANDROID_LOG_ERROR,"MatchPoint3DComProjt","(%f %f) (%f %f)",screenCoord[0],GL_HEIGHT-screenCoord[1],xy1.x,xy1.y);
//			in_point.push_back(xy1);
//			in2_point.push_back(Point2f(screenCoord[0],GL_HEIGHT-screenCoord[1]));
//		}
//	}
//
//    vector<uchar> inliners;
//
//	Mat tmp = findHomography(in_point,in2_point,inliners,CV_RANSAC);
//
//	tmp.convertTo(H,CV_32F);
//	CameraParams camera;
//	camera.aspect = 1;//??? change to 1(1920/1080??=1.77)
//	camera.focal = (dst.size().width * 4.7 / focal_divider) * work_scale/seam_scale;
//	Mat K;
//    camera.K().convertTo(K,CV_32F);
//    Mat R = K.inv() * H.inv() * K;
//	printMatrix(H,"H_MAT");
//	printMatrix(R,"R_MAT");
//	vector<Point2f> src_points;
//	vector<Point2f> dst_points;
//	int inlier_idx = 0;
//	for(int i = 0 ; i < inliners.size() ; i++){
//		if(!inliners[i])
//			continue;
//		Point2f p1 = in_point[i];
//		Point2f p2 = in2_point[i];
//		src_points.push_back(p1);
//		dst_points.push_back(p2);
//		__android_log_print(ANDROID_LOG_ERROR,"MatchPoint3DFF","(%f %f) (%f %f)",p1.x,p1.y,p2.x,p2.y);
//		inlier_idx++;
//	}
//	tmp = findHomography(src_points,dst_points,CV_RANSAC);
//	tmp.convertTo(H,CV_32F);
//	printMatrix(H.inv(),"H");
//	R = K.inv() * H.inv() * K;
//	printMatrix(R,"R_MAT");
//	//minimizeRotation(){}
//	vector<CameraParams> camera_homo(2);
//	for(int i = 0 ; i < 2; i++){
//		CameraParams c;
//		c.aspect = 1;//??? change to 1(1920/1080??=1.77)
//		c.focal = (dst.size().width * 4.7 / focal_divider) * work_scale/seam_scale;
//		c.ppx = 1080/2.0;
//		c.ppy = 1080/2.0;
//		if(i == 0){
//
//			c.R = Mat::eye(3,3,CV_32F);
//		}
//		else{
//			c.R = R;
//		}
//
//		camera_homo[i] = c;
//	}
//	minimizeRotation(src_points,dst_points,camera_homo);
//	printMatrix(camera_homo[0].R,"R_adjust0");
//	printMatrix(camera_homo[1].R,"R_adjusted");
//	//another choice doingbundle on H
//	H = K * R * K.inv();
//	printMatrix(H,"H2_MAT");


}
//may won't work need check
void warpFeature(float warped_image_scale ,vector<CameraParams> cameras,vector<ImageFeatures> features,vector<vector<Point3f>> &out3p){
	float track_work_aspect = tracking_scale/ work_scale;
	Ptr<WarperCreator> warper_creator = new cv::SphericalWarper();
	Ptr<RotationWarper> warper = warper_creator->create(warped_image_scale * track_work_aspect);
	p2d.clear();
	for(int i = 0; i < cameras.size() ;i++){
		vector<Point2f> p2d_per_camera;
		vector<Point3f> p3d_per_camera;
		for(int j = 0; j < features[i].keypoints.size() ;j++){
			Mat k_temp;

			cameras[i].K().convertTo(k_temp,CV_32F);
			Point2f warped_point = warper->warpPoint(features[i].keypoints[j].pt,k_temp,cameras[i].R);
			warped_point.x += work_width*track_work_aspect/2;
			p3d_per_camera.push_back(calc3DPosition(warped_point,track_work_aspect));
			p2d_per_camera.push_back(warped_point);

		}
		p3d.push_back(p3d_per_camera);
		p2d.push_back(p2d_per_camera);
	}

}
inline double calcOpticalDiff(Mat mat1, Mat mat2){
        Mat diff = mat1.inv() * mat2;
        double x = atan2(diff.at<float>(2,1),diff.at<float>(2,2));
        double y = atan2(-diff.at<float>(2,0),sqrt(diff.at<float>(2,1)*diff.at<float>(2,1)+diff.at<float>(2,2)*diff.at<float>(2,2)));
        double z = atan2(diff.at<float>(1,0),diff.at<float>(0,0));
        Mat z_axis = Mat::zeros(3,1,CV_32F);
        z_axis.at<float>(0,0) = 0;
        z_axis.at<float>(1,0) = 0;
        z_axis.at<float>(2,0) = 1;
        Mat z_diff = diff*z_axis;
        //__android_log_print(ANDROID_LOG_INFO,"z_diff","%lf %lf %lf",z_diff.at<float>(0,0),z_diff.at<float>(1,0),z_diff.at<float>(2,0));

        normalize(z_diff,z_diff);
        double cos_z = z_axis.at<float>(0,0) * z_diff.at<float>(0,0) + z_axis.at<float>(1,0) * z_diff.at<float>(1,0) + z_axis.at<float>(2,0) * z_diff.at<float>(2,0);
        return acos(cos_z);

}

inline int isBiggerThanThreshold(Mat mat1, Mat mat2, float threshold){
    Mat diff = mat1.inv() * mat2;
    double x_diff = abs(atan2(diff.at<float>(2,1),diff.at<float>(2,2)));
    double y_diff = abs(atan2(-diff.at<float>(2,0),sqrt(diff.at<float>(2,1)*diff.at<float>(2,1)+diff.at<float>(2,2)*diff.at<float>(2,2))));
    double z_diff = abs(atan2(diff.at<float>(1,0),diff.at<float>(0,0)));
    __android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Diff [%lf %lf %lf]",x_diff,y_diff,z_diff);
    if(x_diff < threshold)
        if(y_diff < threshold)
            if(z_diff < threshold)
                return 0;
    return 1;
}

inline Point3f calc3DPosition(Point2f keypoint,float multiplier){

	float ratio_j = (keypoint.x/(work_width*multiplier));
	float ratio_i = (keypoint.y/(work_height*multiplier));

	float sini = (float) sin(M_PI*ratio_i);
	float cosi = (float) cos(M_PI*ratio_i);
	float sinj = (float) sin(M_PI+(2*M_PI)*1*ratio_j);
	float cosj = (float) cos(M_PI+(2*M_PI)*1*ratio_j);

	//210 = r;
	float x = 210 * sini * sinj;
	float y = 210 * cosi;
	float z = 210 * sini * cosj;

	//xzy ??
	Point3f p(x,y,-z);
	return p;
}


void findWarpForSeam(float warped_image_scale,float seam_scale,float work_scale, vector<ImagePackage> &p_img,vector<CameraParams> cameras){

	double seam_work_aspect = seam_scale/work_scale;
	Ptr<WarperCreator> warper_creator = new cv::SphericalWarper();
//    Ptr<WarperCreator> warper_creator = new cv::CylindricalWarper();
	Ptr<RotationWarper> warper = warper_creator->create(warped_image_scale*seam_work_aspect);
	for(int i = 0; i < p_img.size(); i++){
		if(p_img[i].done != 0){
			continue;
		}
		Mat_<float> K;
		Mat image_warped;
		Mat mask;
		mask.create(p_img[i].image.size(), CV_8U);
		mask.setTo(Scalar::all(255));

		cameras[i].K().convertTo(K, CV_32F);
		float swa = (float)seam_work_aspect;
		K(0,0) *= swa; K(0,2) *= swa; K(1,1) *= swa; K(1,2) *= swa;
		p_img[i].corner = warper->warp(p_img[i].image, K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, image_warped);
		p_img[i].corner;
		warper->warp(mask, K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, p_img[i].mask_warped);
		image_warped.convertTo(p_img[i].image_warped, CV_32F);
		mask.release();
	}
}
unsigned int np2(unsigned int w){
            w--;
			w |= w >> 1;   // Divide by 2^k for consecutive doublings of k up to 32,
            w |= w >> 2;   // and then or the results.
            w |= w >> 4;
            w |= w >> 8;
            w |= w >> 16;
            w++;
            return w;

}
vector<int> findOverlap(vector<Point2i> p,vector<Size> size){
    vector<int> out;
    Rect last(p[p.size()-1],size[size.size()-1]);
    for(int i = 0 ; i < p.size()-1;i++){
        vector<Point2i> p_rect;
        p_rect.push_back(p[i]);
        p_rect.push_back(p[i]+Point(0,size[i].height));
        p_rect.push_back(p[i]+Point(size[i].width,0));
        p_rect.push_back(p[i]+Point(size[i].width,size[i].height));
        for(int j = 0 ;j < p_rect.size();j++){
            if(last.contains(p_rect[j])){
                out.push_back(i);
                break;
            }
        }
    }
    return out;
}
void threadTask(int msg){
    double t = 0;
    for(int i = 0 ; i < 10000000 ;i++){
        t+=((rand()%100) * 0.1);
    }
    __android_log_print(ANDROID_LOG_INFO,"Thread","Hello %d %lf",msg,t);
}

//need to re-done in some part
void doComposition(float warped_image_scale,vector<CameraParams> cameras,vector<ImagePackage> &p_img,Ptr<ExposureCompensator> compensator,float work_scale,float compose_scale,int blend_type,Mat &result,Mat &area){
	cv::FileStorage fs("/sdcard/stitch/position.yml", cv::FileStorage::WRITE);
	double compose_work_aspect = compose_scale / work_scale;
	Mat img_warped;
	Mat dilated_mask, seam_mask;
	Mat mask, mask_warped;
	unsigned char blender_created = 0;
//  Ptr<WarperCreator> warper_creator = new cv::CylindricalWarper();
	Ptr<WarperCreator> warper_creator = new cv::SphericalWarper();
	Ptr<RotationWarper> warper = warper_creator->create(warped_image_scale * compose_work_aspect);
	for (int i = 0; i < p_img.size(); ++i)
	{

		cameras[i].focal *= compose_work_aspect;
		cameras[i].ppx *= compose_work_aspect;
		cameras[i].ppy *= compose_work_aspect;
		if(p_img[i].done != 0 ){
			continue;
		}
		Size sz = p_img[i].full_size;
		if (std::abs(compose_scale - 1) > 1e-1)
		{
			sz.width = cvRound(p_img[i].full_size.width * compose_scale);
			sz.height = cvRound(p_img[i].full_size.height * compose_scale);
		}

		Mat K;
		cameras[i].K().convertTo(K, CV_32F);
		Rect roi = warper->warpRoi(sz, K, cameras[i].R);
		p_img[i].compose_corner = roi.tl();
		p_img[i].compose_size = roi.size();
	}
    double c_feed_total =0;
	for (int i = 0; i < p_img.size(); i++)
	{

		if(p_img[i].done != 0){
		}
		else {
			clock_t c_c0 = std::clock();
			Mat img;
			if (compose_scale - 1 < 0)
				resize(p_img[i].full_image, img, Size(), compose_scale, compose_scale);
			else
				img = p_img[i].full_image;

			Size img_size = img.size();
			Mat K;

//			mask.create(img_size, CV_8U);
//			mask.setTo(Scalar::all(255));

//			warper->warp(mask, K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);
//			p_img[i].compose_mask_warped = seam_mask & mask_warped;
			cameras[i].K().convertTo(K, CV_32F);
			clock_t c_c1 = std::clock();
			Mat xmap,ymap;
			warper->buildMaps(img.size(),K,cameras[i].R,xmap,ymap);
            clock_t c_c15 = std::clock();
            //for(int q = 0; q < img.size();q++){
            //    int x = xmap.at<float>(q);
            //    int y = ymap.at<float>(q);
            //    img_warped.at<float>(q) = img.at<float>(x,y);
            //}
            __android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Type : %d",xmap.type());
            //Run 2 time ?
            remap(img,img_warped,xmap,ymap);
            __android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Step : %d",1);
			//remap(img, img_warped, xmap, ymap, INTER_LINEAR, BORDER_REFLECT);
			//warper->warp(img, K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, img_warped);
			clock_t c_c2 = std::clock();
			img_warped.convertTo(p_img[i].compose_image_warped, CV_8U);
			img.release();
			mask.release();
			__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Step : %d",2);
			dilate(p_img[i].mask_warped, dilated_mask, Mat());
			__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Step : %d",3);
			resize(dilated_mask, seam_mask, img_warped.size());
			__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Step : %d",4);
			img_warped.release();

			p_img[i].compose_mask_warped = seam_mask;
			p_img[i].done = 1;
			clock_t c_c3 = std::clock();
			__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Timer Image Preparing [Resize : %lf] [BuildMap : %lf] [Remap : %lf] [Deliate : %lf]",
								(double)(c_c1-c_c0)/CLOCKS_PER_SEC,(double)(c_c15-c_c1)/CLOCKS_PER_SEC,(double)(c_c2-c_c15)/CLOCKS_PER_SEC,(double)(c_c3-c_c2)/CLOCKS_PER_SEC);

		}
		if (!blender_created)
		{
		    clock_t c_c4 = std::clock();
			blender_created = 1;
			int width = work_width*compose_work_aspect;
			int offset = work_height*compose_work_aspect/2;//??1.18 at 1.73 aspect???
			int height = work_height*compose_work_aspect;//??? 1280
			Rect full(-(width/2),0,width,height);//Sphere(scale=0.2)
			vector<Point> corners(p_img.size());
			vector<Size> sizes(p_img.size());
			for(int i = 0; i < p_img.size() ;i++){
				corners[i] = p_img[i].compose_corner;
				sizes[i] = p_img[i].compose_size;
			    __android_log_print(ANDROID_LOG_DEBUG,"C++","%d : (%d,%d) (%d,%d)",i,corners[i].x,corners[i].y,sizes[i].width,sizes[i].height);
            }
			Rect dst = resultRoi(corners, sizes);
			//Rect dst = full;
			//dst.x = -(width/2) + ((np2((width/2)+dst.x) - (np2((width/2)+dst.x) >> 1)));
            dst.x -= 10;
            dst.y -= 10;
            dst.width += 20;
            dst.height += 20;
			//if(dst.x < 0){
			//    dst.x = -np2(-dst.x);
			//}
			//else{
            //    dst.x = (np2(dst.x) - (np2(dst.x) >> 1));
			//}


			//dst.y = np2(dst.y) - (np2(dst.y) >> 1);
			__android_log_print(ANDROID_LOG_DEBUG,"Test","%d %d",dst.x,dst.y);
            //dst.width = np2(dst.width);
            //dst.height = np2(dst.height);
			composer::prepare(dst);
			area.at<float>(0,0) = (width/2)+dst.x;
			area.at<float>(0,1) = dst.y;
			area.at<float>(0,2) = dst.width;
			area.at<float>(0,3) = dst.height;
			__android_log_print(ANDROID_LOG_INFO,"C++ Composition","Rect full size (%d,%d) (%d,%d)",full.x,full.y,full.width,full.height);
			__android_log_print(ANDROID_LOG_INFO,"C++ Composition","Rect area (%f,%f) (%f,%f)",area.at<float>(0,0),area.at<float>(0,1),area.at<float>(0,2),area.at<float>(0,3));
            clock_t c_c5 = std::clock();
            __android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Timer Blender Create %lf",(double)(c_c5-c_c4)/CLOCKS_PER_SEC);
		}
		__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Debug");
		clock_t c_c6 = std::clock();
		composer::feed(p_img[i].compose_image_warped, p_img[i].compose_mask_warped, p_img[i].compose_corner);
		char warp_file_name[50];
		char mask_file_name[50];
		sprintf(mask_file_name,"/sdcard/stitch/mask%d.jpg",i);
		sprintf(warp_file_name,"/sdcard/stitch/warped%d.jpg",i);
		imwrite(warp_file_name,p_img[i].compose_image_warped);
		imwrite(mask_file_name,p_img[i].compose_mask_warped);
		fs << "c" << p_img[i].compose_corner;
		fs << "s_x" << p_img[i].compose_image_warped.cols;
		fs << "s_y" << p_img[i].compose_image_warped.rows;
		clock_t c_c7 = std::clock();

        c_feed_total+=(double)(c_c7-c_c6);
	}
	__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Timer Feed %lf",c_feed_total/CLOCKS_PER_SEC);

	clock_t c_c8 = std::clock();
	Mat result_mask;
	//opencv doing thing as bgr
	composer::process(result,result_mask);
	clock_t c_c9 = std::clock();
	__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Timer Merged %lf",(double)(c_c9-c_c8)/CLOCKS_PER_SEC);
	//out as rgba
}


JNIEXPORT void JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_nativeAddStitch(JNIEnv*, jobject, jlong imgaddr,jlong rotaddr){

    __android_log_print(ANDROID_LOG_INFO, "C++ AddImage", "Start");
	ImagePackage image_package;
	//Mat& frame  = *(Mat*)imgaddr; Mat full_img;
	Mat& full_img = *(Mat*)imgaddr;
	Mat& rot = *(Mat*)rotaddr;
	transpose(full_img, full_img);
    flip(full_img, full_img,0);
	//GaussianBlur(frame, full_img, cv::Size(0, 0), 3);
    //addWeighted(frame, 1.5, full_img, -0.5, 0, full_img);
	image_package.full_image = full_img;
	image_package.full_size = image_package.full_image.size();
	ImageFeatures feature;
	Mat img;
	char file_str[50]; // enough to hold all numbers up to 64-bits
    sprintf(file_str, "/sdcard/stitch/input%d.jpg", images.size());
//	imwrite(file_str,full_img);
	__android_log_print(ANDROID_LOG_INFO,"C++ AddImage","Recived Full Image Size: %d %d",full_img.size().width,full_img.size().height);
	resize(full_img, img, Size(), work_scale, work_scale);
	findDescriptor(img,feature.keypoints,feature.descriptors);
	feature.img_idx = images.size();
	feature.img_size = img.size();
	resize(full_img, img, Size(), seam_scale, seam_scale);


//    __android_log_print(ANDROID_LOG_VERBOSE,"Feature","SURF Keypoints Size %d",feature.keypoints.size());
//    __android_log_print(ANDROID_LOG_VERBOSE,"Feature","SURF Size (%d %d)",feature.descriptors.cols,feature.descriptors.rows);
//    Ptr<FeaturesFinder> finder = new OrbFeaturesFinder();
//    (*finder)(img, feature);
	__android_log_print(ANDROID_LOG_INFO,"C++ AddImage","Feature Keypoints Size %d",feature.keypoints.size());
	__android_log_print(ANDROID_LOG_INFO,"C++ AddImage","Feature Size (%d %d)",feature.descriptors.cols,feature.descriptors.rows);

	image_package.image = img;
	image_package.size = img.size();
	image_package.feature = feature;
	image_package.done = 0;
    CameraParams camera;
    camera.ppx = camera_ppx * work_scale;
    camera.ppy = camera_ppy * work_scale;
    camera.focal = camera_focal_x * work_scale;
    camera.aspect = camera_focal_y/camera_focal_x;


    //camera.ppx = images[i].feature.img_size.width/2.0;
    //camera.ppy = images[i].feature.img_size.height/2.0;
    //camera.aspect = 1;//??? change to 1(1920/1080??=1.77)
    //camera.focal = (images[i].size.height * 4.7 / 5.2) * work_scale/seam_scale;
    //5.2 - > 10 = bigger
    //4.8 maybe better
    //camera.focal = (images[i].feature.img_size.width * 4.7 / focal_divider);

    camera.R = rot;
    camera.t = Mat::zeros(3,1,CV_32F);
    image_package.param = camera;
    images.push_back(image_package);
}


JNIEXPORT jint JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_nativeStitch(JNIEnv*, jobject,jlong retAddr,jlong areaAddr,jlong rotAddr,jlong refinedRotAddr){

	__android_log_print(ANDROID_LOG_INFO,"C++ Stitching","Start");
	Mat& result = *(Mat*)retAddr;
	Mat& retRot = *(Mat*)rotAddr;
    Mat& refinedRot = *(Mat*)refinedRotAddr;
	//do matcher
    matcher::create(0.3f,6,6);

	int num_images = static_cast<int>(images.size());
	if(num_images < 2){
		return -1;
	}



	int nearest_image = findNearest(0,images.size()-1,images,images[images.size()-1].param.R);
	__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Nearest Image : %d",nearest_image);

	vector<ImageFeatures> features(num_images);
	for(int i = 0; i < num_images; i++){
		features[i] = images[i].feature;
	}
	vector<MatchesInfo> pairwise_matches;

    vector<ImageFeatures> nearest_feature(2);
    nearest_feature[0] = images[nearest_image].feature;
    nearest_feature[1] = images[images.size()-1].feature;
	//change here only doing on new pic

	MatchesInfo matchInfo;
	clock_t c_m1 = clock();
	if(EXPERIMENT){
        matcher::match(nearest_feature, pairwise_matches);
	}else{
	    matcher::match(features, pairwise_matches,images.size()-1);
	}
	clock_t c_m2 = clock();

	vector<CameraParams> cameras;
	for(int i = 0; i < num_images;i++){

		__android_log_print(ANDROID_LOG_INFO,"C++ Stitching","Input Image Size : %d,%d ",images[i].size.height,images[i].size.width);
		cameras.push_back(images[i].param);
	}


	//minimize all pair
//	minimizeRotation(features,pairwise_matches,cameras);
	vector<Point2f> src;
	vector<Point2f> dst;

	for(int i = 0 ; i < pairwise_matches.size();i++){
		__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Pair Number : %d %d %d",i,pairwise_matches[i].src_img_idx,pairwise_matches[i].dst_img_idx);
		if(pairwise_matches[i].src_img_idx == nearest_image && pairwise_matches[i].dst_img_idx == images.size()-1){
			//__android_log_print(ANDROID_LOG_INFO,"C++ Stitching","Nearest Pair %d %d %d",pairwise_matches[i].src_img_idx
			//		,pairwise_matches[i].dst_img_idx,pairwise_matches[i].matches.size());
			if(pairwise_matches[i].matches.size() < 15){
				//return
				__android_log_print(ANDROID_LOG_WARN,"C++ Stitching","Stitch Rejected < 15 matches point..");
				images.pop_back();
				return 0;
			}
			else {
				for (int j = 0; j < pairwise_matches[i].matches.size(); j++) {
					if (pairwise_matches[i].inliers_mask[j]) {
						src.push_back(
						    features[nearest_image].keypoints[pairwise_matches[i].matches[j].queryIdx].pt);
						dst.push_back(features[images.size() -
											   1].keypoints[pairwise_matches[i].matches[j].trainIdx].pt);
					}
				}
			}
		}
	}

	vector<CameraParams> camera_set(2);
	camera_set[0] = cameras[nearest_image];
	camera_set[1] = cameras[images.size()-1];
	Mat comparedRot = cameras[images.size()-1].R.clone();
	__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Minimized Point %d : %d",src.size(),dst.size());
	int iterationCount = minimizeRotation(src,dst,camera_set);
	//Check with ceres minimizer should be best solution
	__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Minimize Iteration %d",iterationCount);
    printMatrix(comparedRot,"Before");
    printMatrix(camera_set[1].R,"After");
	if(isBiggerThanThreshold(comparedRot,camera_set[1].R,0.2)){
        __android_log_print(ANDROID_LOG_WARN,"C++ Stitching","Stitch Rejected > 4 degree..");
        images.pop_back();
		return 0;
	}

	cameras[images.size()-1].R = camera_set[1].R;
    images[images.size()-1].param.R = camera_set[1].R;
	clock_t c_m3 = clock();
	vector<double> focals;
	for (size_t i = 0; i < cameras.size(); ++i)
	{
		focals.push_back(cameras[i].focal);
	}

	sort(focals.begin(), focals.end());
	float warped_image_scale;
	if (focals.size() % 2 == 1)
		warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
	else
		warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;


	work_width = (warped_image_scale ) * M_PI * 2;
	work_height = (((warped_image_scale) / cameras[0].aspect) * M_PI);//??? 1280
	findWarpForSeam(warped_image_scale,seam_scale,work_scale,images,cameras);

	//Create vector of var because need to call seam_finder
	vector<Mat> masks_warped(num_images);
	vector<Mat> images_warped(num_images);
	vector<Point> corners(num_images);
    vector<Size> masks_size(num_images);
    clock_t c_m4 = clock();
#ifdef GRAPHCUT
	for(int i = 0; i < images.size();i++){
		corners[i] = images[i].corner;
        images_warped[i] = images[i].image_warped;
		masks_warped[i] = images[i].mask_warped;
		masks_size[i] = images[i].mask_warped.size();
        __android_log_print(ANDROID_LOG_ERROR,"C++ Stitching","Seam Debug %d %d %d %d %d %d",
        corners[i].x,corners[i].y,images_warped[i].size().width,images_warped[i].size().height,masks_warped[i].size().width,masks_warped[i].size().height);
	}
    Ptr<SeamFinder> seam_finder =  new detail::GraphCutSeamFinder(GraphCutSeamFinderBase::COST_COLOR);
    //This is experiment
    if(!EXPERIMENT){
        seam_finder->find(images_warped,corners,masks_warped);
		__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Full Seamfinder");
    }

    else
    {
    vector<int> overlap_index = findOverlap(corners,masks_size);
    vector<Mat> masks_warped_new(overlap_index.size()+1);
    vector<Mat> images_warped_new(overlap_index.size()+1);
    vector<Point> corners_new(overlap_index.size()+1);
    corners_new[overlap_index.size()] = images[images.size()-1].corner;
    masks_warped_new[overlap_index.size()] = images[images.size()-1].mask_warped;
    images_warped_new[overlap_index.size()] = images[images.size()-1].image_warped;
    for(int i = 0 ; i < overlap_index.size() ;i++){
        corners_new[i] = images[overlap_index[i]].corner;
        masks_warped_new[i] = images[overlap_index[i]].mask_warped;
        images_warped_new[i] = images[overlap_index[i]].image_warped;
        __android_log_print(ANDROID_LOG_ERROR,"C++ Stitching","overlap %d ",overlap_index[i]);
    }
    seam_finder->find(images_warped_new, corners_new, masks_warped_new);
    }
#else
#endif



	clock_t c_m5 = clock();





	for(int i = 0; i < images.size();i++){
	__android_log_print(ANDROID_LOG_ERROR,"C++ Stitching","Seam Debug %d %d %d %d %d %d",
            corners[i].x,corners[i].y,images_warped[i].size().width,images_warped[i].size().height,masks_warped[i].size().width,masks_warped[i].size().height);

    }
	clock_t c_m6 = clock();
//	Mat out;
	Mat& area = *(Mat*)areaAddr;
	doComposition(warped_image_scale,cameras,images,nullptr,work_scale,compose_scale,blend_type,result,area);

	__android_log_print(ANDROID_LOG_ERROR,"C++ Stitching","Composed %d Images",num_images);

	clock_t c_m7 = clock();
	__android_log_print(ANDROID_LOG_INFO,"C++ Stitching,Timer","%lf [Match %lf,Optimize %lf,Warp %lf,Seam %lf,Stitch %lf]",((double)c_m7-c_m1)/CLOCKS_PER_SEC,
						((double)c_m2-c_m1)/CLOCKS_PER_SEC,((double)c_m3-c_m2)/CLOCKS_PER_SEC,((double)c_m4-c_m3)/CLOCKS_PER_SEC,((double)c_m5-c_m4)/CLOCKS_PER_SEC,((double)c_m7-c_m6)/CLOCKS_PER_SEC);
	cv::FileStorage fs("/sdcard/stitch/rotation.yml", cv::FileStorage::WRITE);
    for(int i = 0 ; i < images.size() ; i++){
        fs << "i" << images[i].param.R;
    }
    for(int i = 0; i < 4 ; i++){
        for(int j = 0; j < 4 ;j++){
            if(i == 3 && j == 3){
                refinedRot.at<float>(i,j) = 1;
            }
            else if(i == 3 || j == 3){
                refinedRot.at<float>(i,j) = 0;
            }
            else{
                refinedRot.at<float>(i,j) = images[images.size()-1].param.R.at<float>(i,j);
            }

        }
    }
	return 1;
}
void printMatrix(Mat tmp,string text){
	Mat mat;
	if(mat.type() != CV_32F){
		tmp.convertTo(mat,CV_32F);
	}
	__android_log_print(ANDROID_LOG_VERBOSE, "C++ Stitching", "Matrix %s############################", text.c_str());
	__android_log_print(ANDROID_LOG_VERBOSE, "C++ Stitching", "[%f %f %f]", mat.at<float>(0,0),mat.at<float>(0,1),mat.at<float>(0,2));
	__android_log_print(ANDROID_LOG_VERBOSE, "C++ Stitching", "[%f %f %f]", mat.at<float>(1,0),mat.at<float>(1,1),mat.at<float>(1,2));
	__android_log_print(ANDROID_LOG_VERBOSE, "C++ Stitching", "[%f %f %f]", mat.at<float>(2,0),mat.at<float>(2,1),mat.at<float>(2,2));
	__android_log_print(ANDROID_LOG_VERBOSE, "C++ Stitching", "Matrix ##############################");
}
inline int glhProjectf(float objx, float objy, float objz, float *modelview, float *projection, int *viewport, float *windowCoordinate) {
	//Transformation vectors
	float fTempo[8];
	//Modelview transform
	fTempo[0]=modelview[0]*objx+modelview[4]*objy+modelview[8]*objz+modelview[12];  //w is always 1
	fTempo[1]=modelview[1]*objx+modelview[5]*objy+modelview[9]*objz+modelview[13];
	fTempo[2]=modelview[2]*objx+modelview[6]*objy+modelview[10]*objz+modelview[14];
	fTempo[3]=modelview[3]*objx+modelview[7]*objy+modelview[11]*objz+modelview[15];
	//Projection transform, the final row of projection matrix is always [0 0 -1 0]
	//so we optimize for that.
	fTempo[4]=projection[0]*fTempo[0]+projection[4]*fTempo[1]+projection[8]*fTempo[2]+projection[12]*fTempo[3];
	fTempo[5]=projection[1]*fTempo[0]+projection[5]*fTempo[1]+projection[9]*fTempo[2]+projection[13]*fTempo[3];
	fTempo[6]=projection[2]*fTempo[0]+projection[6]*fTempo[1]+projection[10]*fTempo[2]+projection[14]*fTempo[3];
	fTempo[7]=-fTempo[2];
	//The result normalizes between -1 and 1
	if(fTempo[7]==0.0)	//The w value
		return 0;
	fTempo[7]=1.0/fTempo[7];
	//Perspective division
	fTempo[4]*=fTempo[7];
	fTempo[5]*=fTempo[7];
	fTempo[6]*=fTempo[7];
	//Window coordinates
	//Map x, y to range 0-1
	windowCoordinate[0]=(fTempo[4]*0.5+0.5)*viewport[2]+viewport[0];
	windowCoordinate[1]=(fTempo[5]*0.5+0.5)*viewport[3]+viewport[1];
	//This is only correct when glDepthRange(0.0, 1.0)
	windowCoordinate[2]=(1.0+fTempo[6])*0.5;	//Between 0 and 1
	return 1;
}
JNIEXPORT int JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_nativeKeyFrameSelection(JNIEnv *env,jobject obj,jfloatArray rot){
    jfloat *arr = env->GetFloatArrayElements(rot,0);
    Mat rot_mat(3,3,CV_32F);
    for(int i = 0; i < 3 ; i++){
        for(int j = 0; j < 3 ;j++){
            rot_mat.at<float>(i,j) = arr[i*3+j];
        }
    }
    env->ReleaseFloatArrayElements(rot, arr, JNI_ABORT);
    //Algorithm for automatic captured.
    double min_x = 3.0;
    double min_y = 3.0;
    double min_z = 3.0;
    double min_acos_z = 6.0;
    for(int i = 0 ; i < images.size() ; i++){
        double acos_z  = calcOpticalDiff(rot_mat, images[i].param.R);
        if(abs(acos_z) < abs(min_acos_z)){
            min_acos_z = acos_z;
        }
    }
    //__android_log_print(ANDROID_LOG_INFO,"Z_diff","%lf",min_acos_z*180/M_PI);

    if(min_acos_z > 0.3){
        return 1;
    }
    return 0;
}


