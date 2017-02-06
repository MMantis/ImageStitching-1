

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
//l
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
#define MINIMIZE
#include "stitching.h"

using namespace std;
using namespace cv;
using namespace cv::detail;

void findDescriptor(Mat img,std::vector<KeyPoint> &keypoints ,UMat &descriptor, Mat &des){
	if(detector_setup){
		//for surf
//		detector->set("hessianThreshold", 300);
//		detector->set("nOctaves", 3);
//		detector->set("nOctaveLayers", 4);
		detector_setup = 0;
	}
//	ORB orb;
//	SurfFeatureDetector surf;
//	FREAK extractor;

	Mat gray_img;
	Mat mask;
	mask.create(img.size(), CV_8U);
	mask.setTo(Scalar::all(255));
	int x_margin = img.rows/3;
	int y_margin = img.cols/3;
	cvtColor(img, gray_img, CV_BGR2GRAY);
	__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","1");
	detector->detectAndCompute(gray_img , mask, keypoints, des);


	__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","2");
//	surf.detect(gray_img, keypoints,mask);
//	orb.detect(gray_img, keypoints,mask);
//	extractor.compute(gray_img,keypoints,descriptor);
	descriptor = des.reshape(1, static_cast<int>(keypoints.size())).getUMat(ACCESS_RW);
	Mat out;
#ifdef DEBUG
	drawKeypoints(img,keypoints,out);
    char file_str[50]; // enough to hold all numbers up to 64-bits
    sprintf(file_str, "/sdcard/stitch/keypoint%d.jpg", images.size());
    imwrite(file_str,out);
#endif

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

inline void vectorMatrixMultiply(float vec[],float matrix[], float *out ){
	out[0] = matrix[0] * vec[0] + matrix[1] * vec[1] + matrix[2] * vec[2];
	out[1] = matrix[3] * vec[0] + matrix[4] * vec[1] + matrix[5] * vec[2];
	out[2] = matrix[6] * vec[0] + matrix[7] * vec[1] + matrix[8] * vec[2];
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
		if(0){
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

Rect findResultROI(vector<Rect> rects){
	Point tl(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
	Point br(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
	for(int i = 0 ; i < rects.size();i++){
		tl.x = std::min(tl.x, rects[i].x);
		tl.y = std::min(tl.y, rects[i].y);
		br.x = std::max(br.x, rects[i].x + rects[i].width);
		br.y = std::max(br.y, rects[i].y + rects[i].height);
	}
	Rect r(tl,br);
	r.y += 20;
	return r;
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


vector<Rect> findROI(float warped_image_scale,std::vector<ImagePackage> p_img){
	double compose_work_aspect = compose_scale / work_scale;
	clock_t c_t0 = std::clock();
	vector<Rect> rois;
	float u,v,w;
	int count = 0;
	float vec[3] = {0,0,1};
	Mat R,K,K_inv;
	vector<Mat> warpped_borders(p_img.size());
	float r_kinv[9] = {0};
	float out[9];
	for(int i = 0; i < p_img.size();i++){
		warpped_borders[i] = Mat(1,(p_img[i].full_image.rows*2) + (p_img[i].full_image.cols*2),CV_8UC3);
		float tl_x = MAXFLOAT;
		float tl_y = MAXFLOAT;
		float br_x = -MAXFLOAT;
		float br_y = -MAXFLOAT;
		p_img[i].param.R.convertTo(R,CV_32F);
		CameraParams param = p_img[i].param;
		param.focal *= compose_work_aspect;
		param.ppx *= compose_work_aspect;
		param.ppy *= compose_work_aspect;
		__android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","Image type : %d",p_img[i].full_image.type());

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
		__android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI"," Size : %d %d",p_img[i].full_image.cols,p_img[i].full_image.rows);
		for(int x = 0 ; x < p_img[i].full_image.rows;x++) {
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
			warpped_borders[i].at<Point3_<uchar>>(0, counter) = p_img[i].full_image.at<Point3_<uchar>>(0, x);
			counter++;
		}
		__android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","1 sizey %d",p_img[i].full_image.rows);
		for(int x = 0 ; x < p_img[i].full_image.rows;x++) {
		//right
			vec[0] = p_img[i].full_image.cols-1;
			vec[1] = x;
			vectorMatrixMultiply(vec,r_kinv,out);
			u = warped_image_scale*compose_work_aspect * atan2f(out[0], out[2]);
			w = out[1] / sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
			v = warped_image_scale*compose_work_aspect * (static_cast<float>(CV_PI) - acosf(w == w ? w : 0));

			if(u < tl_x) tl_x = u;
			if(v < tl_y) tl_y = v;
			if(u > br_x) br_x = u;
			if(v > br_y) br_y = v;
			warpped_borders[i].at<Point3_<uchar>>(0, counter) = p_img[i].full_image.at<Point3_<uchar>>(p_img[i].full_image.cols-1, x);
			counter++;
		}

		__android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","2 , size %d",p_img[i].full_image.total());
		for(int y = 0 ; y< p_img[i].full_image.cols ;y++) {
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

			warpped_borders[i].at<Point3_<uchar>>(0, counter) = p_img[i].full_image.at<Point3_<uchar>>(y, 0);
			counter++;
		}

		__android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","3 , sizey %d",p_img[i].full_image.cols);
		for(int y = 0 ; y< p_img[i].full_image.cols ;y++) {
			//bottom
			vec[0] = y;
			vec[1] = p_img[i].full_image.rows-1;
			vectorMatrixMultiply(vec,r_kinv,out);
			u = warped_image_scale*compose_work_aspect * atan2f(out[0], out[2]);
			w = out[1] / sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
			v = warped_image_scale*compose_work_aspect * (static_cast<float>(CV_PI) - acosf(w == w ? w : 0));

			if(u < tl_x) tl_x = u;
			if(v < tl_y) tl_y = v;
			if(u > br_x) br_x = u;
			if(v > br_y) br_y = v;
			warpped_borders[i].at<Point3_<uchar>>(0, counter) = p_img[i].full_image.at<Point3_<uchar>>(y, p_img[i].full_image.rows-1);
			counter++;
		}

		__android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","4");

		__android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","%f %f %f %f",tl_x,tl_y,br_x,br_y);
		rois.push_back(Rect(Point(tl_x,tl_y),Point(br_x,br_y)));
		__android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI","Last pixel %d : %u %u %u",i,warpped_borders[i].at<Point3_<uchar>>(0,counter-1).x,
		warpped_borders[i].at<Point3_<uchar>>(0,counter-1).y,warpped_borders[i].at<Point3_<uchar>>(0,counter-1).z);
	}
	clock_t c_t1 = std::clock();
	__android_log_print(ANDROID_LOG_DEBUG,"C++ FindROI"," %lf %d",(double)(c_t1-c_t0)/CLOCKS_PER_SEC,count);

	return rois;
}

void doComposition(float warped_image_scale,vector<CameraParams> cameras,vector<ImagePackage> &p_img,Ptr<ExposureCompensator> compensator,float work_scale,float compose_scale,int blend_type,Mat &result,Mat &area){

#ifdef DEBUG
	cv::FileStorage pfs("/sdcard/stitch/position.yml", cv::FileStorage::WRITE);
#endif

	double compose_work_aspect = compose_scale / work_scale;
	Mat img_warped;
	Mat dilated_mask, seam_mask;
	Mat mask, mask_warped;
	unsigned char blender_created = 0;
//  Ptr<WarperCreator> warper_creator = new cv::CylindricalWarper();
	Ptr<WarperCreator> warper_creator = new cv::SphericalWarper();
	Ptr<RotationWarper> warper = warper_creator->create(warped_image_scale * compose_work_aspect);
	__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Warp Scale : %f",warped_image_scale*compose_work_aspect);

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

		if(0){
		}
		else {
			clock_t c_c0 = std::clock();
			Mat img;
			if (compose_scale - 1 < 0)
				resize(p_img[i].full_image, img, Size(), compose_scale, compose_scale);
			else
				img = p_img[i].full_image;

			Size img_size = img.size();
			Mat xmap,ymap;
			Mat K;
			cameras[i].K().convertTo(K, CV_32F);
			clock_t c_c1 = std::clock();

			Rect r = warper->buildMaps(img.size(),K,cameras[i].R,xmap,ymap);
#ifdef DEBUG
			char map_file_name[50];
			sprintf(map_file_name,"/sdcard/stitch/map%d.yml",i);
			cv::FileStorage mapfs(map_file_name, cv::FileStorage::WRITE);
			SphericalProjector projector;
			projector.scale = warped_image_scale*compose_work_aspect;
			projector.setCameraParams(K,cameras[i].R);
			__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Rect %d %d %d %d",r.x,r.y,r.width,r.height);
			mapfs << "k_rinv" << Mat(3,3,CV_32F,projector.k_rinv);
			mapfs << "r_kinv" << Mat(3,3,CV_32F,projector.r_kinv);
			char img_file_name[50];
			sprintf(img_file_name,"/sdcard/stitch/img%d.jpg",i);
			imwrite(img_file_name,img);
#endif

			__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Rect tl (%d,%d) br (%d,%d)",r.x,r.y,r.br().x,r.br().y);
			clock_t c_c15 = std::clock();
			remap(img,img_warped,xmap,ymap);
			__android_log_print(ANDROID_LOG_DEBUG,"C++ Composition","Step : %d",1);
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
			dst.x -= 10;
			dst.y -= 10;
			dst.width += 20;
			dst.height += 20;
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
#ifdef DEBUG
		char warp_file_name[50];
		char mask_file_name[50];
		sprintf(mask_file_name,"/sdcard/stitch/mask%d.jpg",i);
		sprintf(warp_file_name,"/sdcard/stitch/warped%d.jpg",i);
		imwrite(warp_file_name,p_img[i].compose_image_warped);
		imwrite(mask_file_name,p_img[i].compose_mask_warped);
		pfs << "c" << p_img[i].compose_corner;
		pfs << "s_x" << p_img[i].compose_image_warped.cols;
		pfs << "s_y" << p_img[i].compose_image_warped.rows;
#endif
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
	//GaussianBlur(frame, full_img, cv::Size(0, 0), 3);
	//addWeighted(frame, 1.5, full_img, -0.5, 0, full_img);
	image_package.full_image = full_img;
	image_package.full_size = image_package.full_image.size();
	ImageFeatures feature;
	Mat img;
#ifdef DEBUG
	char file_str[50]; // enough to hold all numbers up to 64-bits
  	sprintf(file_str, "/sdcard/stitch/input%d.jpg", images.size());
	imwrite(file_str,full_img);
#endif
	__android_log_print(ANDROID_LOG_INFO,"C++ AddImage","Recived Full Image Size: %d %d",full_img.size().width,full_img.size().height);
	resize(full_img, img, Size(), work_scale, work_scale);
#ifdef MINIMIZE
	findDescriptor(img,feature.keypoints,feature.descriptors,image_package.feature_desc);
	feature.img_idx = images.size();
	feature.img_size = img.size();
	resize(full_img, img, Size(), seam_scale, seam_scale);
#endif


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

	camera.R = rot;
	camera.t = Mat::zeros(3,1,CV_32F);
	image_package.param = camera;
	images.push_back(image_package);
}


JNIEXPORT jint JNICALL Java_com_kunato_imagestitching_ImageStitchingNative_nativeStitch(JNIEnv*, jobject,jlong retAddr,jlong areaAddr,jlong rotAddr,jlong refinedRotAddr,jlong roiAddr,jlong k_rinvAddr) {
	cv::FileStorage rfs("/sdcard/stitch/rotation.yml", cv::FileStorage::WRITE);
	__android_log_print(ANDROID_LOG_INFO, "C++ Stitching", "Start");
	Mat &result = *(Mat *) retAddr;
	Mat &retRot = *(Mat *) rotAddr;
	Mat &refinedRot = *(Mat *) refinedRotAddr;
	Mat &roiRot = *(Mat *) roiAddr;
	Mat &k_rinvRot = *(Mat *) k_rinvAddr;
	Mat &area = *(Mat *) areaAddr;

	//do matcher
	matcher::create(0.3f, 6, 6);

	int num_images = static_cast<int>(images.size());
	if (num_images < 2) {
		return -1;
	}


	int nearest_image = findNearest(0, images.size() - 1, images,
									images[images.size() - 1].param.R);
	__android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "Nearest Image : %d", nearest_image);

	vector<ImageFeatures> features(num_images);
	for (int i = 0; i < num_images; i++) {
		features[i] = images[i].feature;
	}
	vector<MatchesInfo> pairwise_matches;

	vector<ImageFeatures> nearest_feature(2);
	nearest_feature[0] = images[nearest_image].feature;
	nearest_feature[1] = images[images.size() - 1].feature;
	//change here only doing on new pic

	MatchesInfo matchInfo;
	clock_t c_m1 = clock();
#ifdef MINIMIZE
	if (EXPERIMENT) {
		matcher::match(nearest_feature, pairwise_matches);
	} else {
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "1");
		matcher::match(features, pairwise_matches, images.size() - 1);

		int matched_index;
		for(int i = 0 ; i < pairwise_matches.size() ;i++){
			if(pairwise_matches[i].dst_img_idx == images.size()-1 && pairwise_matches[i].src_img_idx == nearest_image) {
				matched_index = i;
				break;
			}
		}
		__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","matched_inliners_size_original : %d",pairwise_matches[matched_index].num_inliers);

		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
		vector<vector<DMatch>> matched_descriptors;
		Mat desc1,desc2;

		__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Test1");
		matcher->knnMatch(images[nearest_image].feature_desc,images[images.size()-1].feature_desc,matched_descriptors,2);

		__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Test2");
		vector<DMatch> matched_pair;
		vector<KeyPoint> matched1,matched2;
		Mat homography;
		//pairwise_matches[matched_index].H.convertTo(homography,CV_32F);
		Mat K;
		images[nearest_image].param.K().convertTo(K,CV_32F);
		K.at<float>(0,2) = K.at<float>(1,2) = 0;
		printMatrix(K,"Ktest");

		homography = K * images[images.size()-1].param.R.inv() * images[nearest_image].param.R * K.inv();
		for(int i = 0 ; i < matched_descriptors.size();i++){
			DMatch first = matched_descriptors[i][0];
			float dist1 = matched_descriptors[i][0].distance;
			float dist2 = matched_descriptors[i][1].distance;
			//?
			if(dist1 < 0.8f * dist2){
				matched1.push_back(features[nearest_image].keypoints[first.queryIdx]);
				matched2.push_back(features[images.size()-1].keypoints[first.trainIdx]);
			}
			matched_pair.push_back(first);
		}

		__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Test3");
		vector<uchar> matched_inliners(matched1.size());
		printMatrix(homography,"Homography");
		for(int i = 0; i < matched1.size() ;i++){
			Mat col = Mat::ones(3,1,CV_32F);
			col.at<float>(0) = matched1[i].pt.x;
			col.at<float>(1) = matched1[i].pt.y;
			col = homography * col;
			//col = homography * col;
			col /= col.at<float>(2);

			float dist = sqrt( ((col.at<float>(0) - matched2[i].pt.x ) * ( col.at<float>(0) - matched2[i].pt.x)) + ((col.at<float>(1) - matched2[i].pt.y ) * (col.at<float>(1) - matched2[i].pt.y)));

			//?
			__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","matched data dist %f",dist);
			if(dist < 80){
				matched_inliners[i] = true;
			}
			else{
				matched_inliners[i] = false;
			}
		}
		int matched_inliner_size = 0;
		for(int i = 0;  i < matched_inliners.size() ;i++){
			if(matched_inliners[i] == true){
				matched_inliner_size+=1;
			}
		}

		__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Test4");
		__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","matched_inliners_original_compare %d %d",pairwise_matches[matched_index].inliers_mask.size(),pairwise_matches[matched_index].num_inliers);
		//pairwise_matches[matched_index].inliers_mask = matched_inliners;
		//pairwise_matches[matched_index].num_inliers = matched_inliner_size;

		//pairwise_matches[matched_index].matches = matched_pair;

		__android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","matched_inliners_size : %d %d",matched_inliners.size(),matched_inliner_size);


		__android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "2");
	}
#endif
	clock_t c_m2 = clock();

	vector<CameraParams> cameras;
	for (int i = 0; i < num_images; i++) {
		__android_log_print(ANDROID_LOG_INFO, "C++ Stitching", "Input Image Size : %d,%d ",
							images[i].size.height, images[i].size.width);
		cameras.push_back(images[i].param);
	}
#ifdef DEBUG_MATCHES
	int pair_index = -1;
    Mat match_debug;
    for(int i = 0 ; i < pairwise_matches.size();i++){
        if(pairwise_matches[i].src_img_idx == nearest_image && pairwise_matches[i].dst_img_idx == images.size()-1) {
            pair_index = i;
        }

    }
    __android_log_print(ANDROID_LOG_DEBUG,"C++ Stitching","Pair Index %d",pair_index);
    drawMatches(images[nearest_image].image,nearest_feature[0].keypoints,images[images.size()-1].image,nearest_feature[1].keypoints,pairwise_matches[pair_index].matches,match_debug);
    char file_str[50]; // enough to hold all numbers up to 64-bits
    sprintf(file_str, "/sdcard/stitch/matches%d.jpg", images.size());
    imwrite(file_str, match_debug);
#endif




	//minimize all pair
	vector<Point2f> src;
	vector<Point2f> dst;

	for (int i = 0; i < pairwise_matches.size(); i++) {
		__android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "Pair Number : %d %d %d", i,
							pairwise_matches[i].src_img_idx, pairwise_matches[i].dst_img_idx);
		if (pairwise_matches[i].src_img_idx == nearest_image &&
			pairwise_matches[i].dst_img_idx == images.size() - 1) {
			//__android_log_print(ANDROID_LOG_INFO,"C++ Stitching","Nearest Pair %d %d %d",pairwise_matches[i].src_img_idx
			//		,pairwise_matches[i].dst_img_idx,pairwise_matches[i].matches.size());
			if (pairwise_matches[i].num_inliers < 15) {
				//return
				__android_log_print(ANDROID_LOG_WARN, "C++ Stitching",
									"Stitch Rejected < 15 matches point..");
				images.pop_back();
				return 0;
			}
			else {
				for (int j = 0; j < pairwise_matches[i].inliers_mask.size(); j++) {
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
	vector<Mat> copy(images.size());
	for(int i = 0 ; i < images.size();i++){
		copy[i] = images[i].param.R.clone();
	}
	for(int z = 1 ; z < 2 ; z++){
		for (int i = 0; i < images.size(); i++) {
			images[i].param.R = copy[i];
			cameras[i].R = copy[i];

			rfs << "i1" << images[i].param.R;
		}
		vector<CameraParams> camera_set(2);
		camera_set[0] = cameras[nearest_image];
		camera_set[1] = cameras[images.size() - 1];
		Mat comparedRot = cameras[images.size() - 1].R.clone();
		__android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "Minimized Point %d : %d", src.size(),
							dst.size());
		vector<Mat> minimized_R(cameras.size());
#ifdef MINIMIZE
#ifndef MINIMIZE_GLOBAL
		int iterationCount = minimizeRotation(src, dst, camera_set, z);
		__android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "Minimize Iteration %d",
							iterationCount);

//		if (isBiggerThanThreshold(comparedRot, camera_set[1].R, 0.2)) {
//			__android_log_print(ANDROID_LOG_WARN, "C++ Stitching", "Stitch Rejected > 4 degree..");
//			images.pop_back();
//			return 0;
//		}
		cameras[images.size() - 1].R = camera_set[1].R;
		images[images.size() - 1].param.R = camera_set[1].R;

#else
		vector<Mat> compareRots(cameras.size());
    for(int i = 0 ; i < cameras.size();i++){
        compareRots[i] = cameras[i].R;
    }
    minimizeRotation(features,pairwise_matches,cameras);
    for(int i = 0 ; i < cameras.size() ;i++){
        if(isBiggerThanThreshold(compareRots[i],cameras[i].R,0.2)){
            __android_log_print(ANDROID_LOG_WARN,"C++ Stitching","Stitch Rejected > 4 degree..");
            images.pop_back();
            return 0;
        }
    }
    for(int i = 0; i < cameras.size() ;i++){
        //Don't forget to update images[i].param.R
        images[i].param.R = cameras[i].R;
    }
#endif
#endif
		//Check with ceres minimizer should be best solution
//    printMatrix(comparedRot,"Before");
//    printMatrix(camera_set[1].R,"After");



		clock_t c_m3 = clock();
		vector<double> focals;
		for (size_t i = 0; i < cameras.size(); ++i) {
			focals.push_back(cameras[i].focal);
		}

		sort(focals.begin(), focals.end());
		float warped_image_scale;
		if (focals.size() % 2 == 1)
			warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
		else
			warped_image_scale =
					static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) *
					0.5f;

		roiRot.create(images.size(), 4, CV_32S);
		k_rinvRot.create(images.size() * 3, 3, CV_32F);
		vector<Rect> rects = findROI(warped_image_scale, images);
		Mat k_temp, rinv_temp, rinv32_temp;
		double compose_work_aspect = compose_scale / work_scale;
		for (int i = 0; i < images.size(); i++) {
			rinv_temp = images[i].param.R.inv();
			CameraParams param = images[i].param;
			param.focal *= compose_work_aspect;
			param.ppx *= compose_work_aspect;
			param.ppy *= compose_work_aspect;
			param.K().convertTo(k_temp, CV_32F);
			rinv_temp.convertTo(rinv32_temp, CV_32F);
			Mat k_rinv = k_temp * rinv32_temp;
			printMatrix(k_temp, "K_RINV1");
			printMatrix(rinv_temp, "K_RINV2");
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					k_rinvRot.at<float>(i * 3 + j, k) = k_rinv.at<float>(j, k);
				}
			}
			__android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "ROI %d %d %d %d", rects[i].x,
								rects[i].y, rects[i].width, rects[i].height);
			roiRot.at<int>(i, 0) = rects[i].x;
			roiRot.at<int>(i, 1) = rects[i].y;
			roiRot.at<int>(i, 2) = rects[i].width;
			roiRot.at<int>(i, 3) = rects[i].height;
		}
		Rect result_rect = findResultROI(rects);
		work_width = (warped_image_scale) * M_PI * 2;

		area.at<float>(0, 0) = (work_width * compose_work_aspect / 2) + result_rect.x;
		area.at<float>(0, 1) = result_rect.y;
		area.at<float>(0, 2) = result_rect.width;
		area.at<float>(0, 3) = result_rect.height;

		__android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "Area : %f %f %f %f",
							area.at<float>(0, 0), area.at<float>(0, 1), area.at<float>(0, 2),
							area.at<float>(0, 3));

		work_height = (((warped_image_scale) / cameras[0].aspect) * M_PI);//??? 1280

#ifdef CPU_STITCH
		findWarpForSeam(warped_image_scale,seam_scale,work_scale,images,cameras);
#endif

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

		rfs << "k" << images[0].param.K();
		for(int i = 0 ; i < images.size() ; i++){
			rfs << "i2" << cameras[i].R;
		}
		clock_t c_m5 = clock();
		clock_t c_m6 = clock();
#ifdef CPU_STITCH
		doComposition(warped_image_scale,cameras,images,nullptr,work_scale,compose_scale,blend_type,result,area);
		Mat cvtRGB;
		char result_str[50];
		sprintf(result_str,"/sdcard/stitch/cpu_panorama%d%d.jpg",num_images,z);
		cvtColor(result,cvtRGB,COLOR_BGRA2RGB);
		imwrite(result_str,cvtRGB);
		__android_log_print(ANDROID_LOG_ERROR,"C++ Stitching","Composed %d Images",num_images);
#endif

	}
	clock_t c_m7 = clock();
//	__android_log_print(ANDROID_LOG_INFO,"C++ Stitching,Timer","%lf [Match %lf,Optimize %lf,Warp %lf,Seam %lf,Stitch %lf]",((double)c_m7-c_m1)/CLOCKS_PER_SEC,
//						((double)c_m2-c_m1)/CLOCKS_PER_SEC,((double)c_m3-c_m2)/CLOCKS_PER_SEC,((double)c_m4-c_m3)/CLOCKS_PER_SEC,((double)c_m5-c_m4)/CLOCKS_PER_SEC,((double)c_m7-c_m6)/CLOCKS_PER_SEC);

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
	__android_log_print(ANDROID_LOG_VERBOSE, "C++ Stitching", "%s [%f %f %f]", text.c_str() , mat.at<float>(0,0),mat.at<float>(0,1),mat.at<float>(0,2));
	__android_log_print(ANDROID_LOG_VERBOSE, "C++ Stitching", "%s [%f %f %f]", text.c_str() , mat.at<float>(1,0),mat.at<float>(1,1),mat.at<float>(1,2));
	__android_log_print(ANDROID_LOG_VERBOSE, "C++ Stitching", "%s [%f %f %f]", text.c_str() , mat.at<float>(2,0),mat.at<float>(2,1),mat.at<float>(2,2));
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
		return 0;
	}
	return 0;
}


