//
// Created by kunato on 2/15/17.
//

#include <android/log.h>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/calib3d.hpp>
#include "BundleCV.h"
#include "util.h"
int total_num_matches_;

int num_params_per_cam_ = 4;
int num_errs_per_measurement_ = 3;
Mat err1_, err2_;
Mat cam_params_;
Mat rot_params_;
TermCriteria term_criteria_(TermCriteria::EPS + TermCriteria::COUNT, 1000, DBL_EPSILON);

vector<Point2f> src_;

vector<Point2f> dst_;


Size size_;

bool focal_estimate_;

void setUpInitialCameraParams(const std::vector<CameraParams> &cameras)
{
    if(focal_estimate_) {
        cam_params_.create(4, 1, CV_64F);
        rot_params_.create(4, 1, CV_64F);
        SVD svd;
        cam_params_.at<double>(0, 0) = cameras[1].focal;
        rot_params_.at<double>(0, 0) = cameras[0].focal;
        for (int i = 0; i < 2; ++i) {
            svd(cameras[i].R, SVD::FULL_UV);
            Mat R = svd.u * svd.vt;
            if (determinant(R) < 0)
                R *= -1;

            Mat rvec;
            Rodrigues(R, rvec);
            CV_Assert(rvec.type() == CV_32F);
            if (i == 1) {
                cam_params_.at<double>(1 + 0, 0) = rvec.at<float>(0, 0);
                cam_params_.at<double>(1 + 1, 0) = rvec.at<float>(1, 0);
                cam_params_.at<double>(1 + 2, 0) = rvec.at<float>(2, 0);
            }
            if (i == 0) {
                rot_params_.at<double>(1 + 0, 0) = rvec.at<float>(0, 0);
                rot_params_.at<double>(1 + 1, 0) = rvec.at<float>(1, 0);
                rot_params_.at<double>(1 + 2, 0) = rvec.at<float>(2, 0);

            }
        }
    }
    else{
        cam_params_.create(3, 1, CV_64F);
        rot_params_.create(4, 1, CV_64F);
        SVD svd;
        rot_params_.at<double>(0, 0) = cameras[0].focal;
        for (int i = 0; i < 2; ++i) {
            svd(cameras[i].R, SVD::FULL_UV);
            Mat R = svd.u * svd.vt;
            if (determinant(R) < 0)
                R *= -1;
            Mat rvec;
            Rodrigues(R, rvec);
            CV_Assert(rvec.type() == CV_32F);
            if (i == 1) {
                cam_params_.at<double>(0, 0) = rvec.at<float>(0, 0);
                cam_params_.at<double>(1, 0) = rvec.at<float>(1, 0);
                cam_params_.at<double>(2, 0) = rvec.at<float>(2, 0);
            }
            if (i == 0) {
                rot_params_.at<double>(1 + 0, 0) = rvec.at<float>(0, 0);
                rot_params_.at<double>(1 + 1, 0) = rvec.at<float>(1, 0);
                rot_params_.at<double>(1 + 2, 0) = rvec.at<float>(2, 0);

            }
        }
    }
}


void obtainRefinedCameraParams(std::vector<CameraParams> &cameras)
{
    if(focal_estimate_) {
        cameras[1].focal = cam_params_.at<double>(0, 0);

        __android_log_print(ANDROID_LOG_DEBUG, "C++ BundleCV", "focal %d : %lf", 0,
                            cameras[0].focal);
        __android_log_print(ANDROID_LOG_DEBUG, "C++ BundleCV", "focal %d : %lf", 1,
                            cameras[1].focal);

        for (int i = 0; i < 1; ++i) {
            Mat rvec(3, 1, CV_64F);
            rvec.at<double>(0, 0) = cam_params_.at<double>(1 + i * 3 + 0, 0);
            rvec.at<double>(1, 0) = cam_params_.at<double>(1 + i * 3 + 1, 0);
            rvec.at<double>(2, 0) = cam_params_.at<double>(1 + i * 3 + 2, 0);
            Rodrigues(rvec, cameras[i].R);

            Mat tmp;
            cameras[1].R.convertTo(tmp, CV_32F);
            cameras[1].R = tmp;

        }
    }
    else{
        cameras[1].focal = rot_params_.at<double>(0, 0);

        __android_log_print(ANDROID_LOG_DEBUG, "C++ BundleCV", "focal %d : %lf", 0,
                            cameras[0].focal);
        __android_log_print(ANDROID_LOG_DEBUG, "C++ BundleCV", "focal %d : %lf", 1,
                            cameras[1].focal);

        for (int i = 0; i < 1; ++i) {
            Mat rvec(3, 1, CV_64F);
            rvec.at<double>(0, 0) = cam_params_.at<double>(0, 0);
            rvec.at<double>(1, 0) = cam_params_.at<double>(1, 0);
            rvec.at<double>(2, 0) = cam_params_.at<double>(2, 0);
            Rodrigues(rvec, cameras[i].R);

            Mat tmp;
            cameras[1].R.convertTo(tmp, CV_32F);
            cameras[1].R = tmp;

        }
    }
}


void calcError(Mat &err)
{
    err.create(total_num_matches_ * 3, 1, CV_64F);

    int match_idx = 0;
        double f1 = rot_params_.at<double>(0, 0);
        double f2 = cam_params_.at<double>(0, 0);

        double R1[9];
        Mat R1_(3, 3, CV_64F, R1);
        Mat rvec(3, 1, CV_64F);
        rvec.at<double>(0, 0) = rot_params_.at<double>(1, 0);
        rvec.at<double>(1, 0) = rot_params_.at<double>(2, 0);
        rvec.at<double>(2, 0) = rot_params_.at<double>(3, 0);
        Rodrigues(rvec, R1_);

        double R2[9];
        Mat R2_(3, 3, CV_64F, R2);
        rvec.at<double>(0, 0) = cam_params_.at<double>(1, 0);
        rvec.at<double>(1, 0) = cam_params_.at<double>(2, 0);
        rvec.at<double>(2, 0) = cam_params_.at<double>(3, 0);
        Rodrigues(rvec, R2_);


        Mat_<double> K1 = Mat::eye(3, 3, CV_64F);
        K1(0,0) = f1; K1(0,2) = size_.width * 0.5;
        K1(1,1) = f1; K1(1,2) = size_.height * 0.5;

        Mat_<double> K2 = Mat::eye(3, 3, CV_64F);
        K2(0,0) = f2; K2(0,2) = size_.width * 0.5;
        K2(1,1) = f2; K2(1,2) = size_.height * 0.5;

        Mat_<double> H1 = R1_ * K1.inv();
        Mat_<double> H2 = R2_ * K2.inv();

        for (size_t k = 0; k < src_.size(); ++k)
        {


            Point2f p1 = src_[k];
            double x1 = H1(0,0)*p1.x + H1(0,1)*p1.y + H1(0,2);
            double y1 = H1(1,0)*p1.x + H1(1,1)*p1.y + H1(1,2);
            double z1 = H1(2,0)*p1.x + H1(2,1)*p1.y + H1(2,2);
            double len = std::sqrt(x1*x1 + y1*y1 + z1*z1);
            x1 /= len; y1 /= len; z1 /= len;

            Point2f p2 = dst_[k];
            double x2 = H2(0,0)*p2.x + H2(0,1)*p2.y + H2(0,2);
            double y2 = H2(1,0)*p2.x + H2(1,1)*p2.y + H2(1,2);
            double z2 = H2(2,0)*p2.x + H2(2,1)*p2.y + H2(2,2);
            len = std::sqrt(x2*x2 + y2*y2 + z2*z2);
            x2 /= len; y2 /= len; z2 /= len;

            double mult = std::sqrt(f1 * f2);
            err.at<double>(3 * match_idx, 0) = mult * (x1 - x2);
            err.at<double>(3 * match_idx + 1, 0) = mult * (y1 - y2);
            err.at<double>(3 * match_idx + 2, 0) = mult * (z1 - z2);

            match_idx++;
        }

}


void calcJacobian(Mat &jac)
{
    if(focal_estimate_) {
        jac.create(total_num_matches_ * 3, 4, CV_64F);

        double val;
        const double step = 1e-3;

        for (int i = 0; i < 4; ++i) {
            val = cam_params_.at<double>(i, 0);
            cam_params_.at<double>(i, 0) = val - step;
            calcError(err1_);
            cam_params_.at<double>(i, 0) = val + step;
            calcError(err2_);
            calcDeriv(err1_, err2_, 2 * step, jac.col(i));
            cam_params_.at<double>(i, 0) = val;


        }
    }else{
        jac.create(total_num_matches_ * 3, 3, CV_64F);

        double val;
        const double step = 1e-3;

        for (int i = 0; i < 3; ++i) {
            val = cam_params_.at<double>(i, 0);
            cam_params_.at<double>(i, 0) = val - step;
            calcError(err1_);
            cam_params_.at<double>(i, 0) = val + step;
            calcError(err2_);
            calcDeriv(err1_, err2_, 2 * step, jac.col(i));
            cam_params_.at<double>(i, 0) = val;


        }
    }
}


bool estimate_(vector<Point2f> src,vector<Point2f> dst,Size size,
               std::vector<CameraParams> &cameras, bool focal_estimate)
{
    src_ = src;
    dst_ = dst;
    size_ = size;
    focal_estimate_ = focal_estimate;
    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","1");
    setUpInitialCameraParams(cameras);

    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","2");

    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","3");
    // Compute number of correspondences
    total_num_matches_ = 0;
        total_num_matches_ = static_cast<int>(src.size());

    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","3");
    int param_number = 3;
    if(focal_estimate_){
        param_number = 4;
    }
    CvLevMarq solver(param_number,
                     total_num_matches_ * num_errs_per_measurement_,
                     term_criteria_);
    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","3.5");
    Mat err, jac;
    CvMat matParams = cam_params_;
    cvCopy(&matParams, solver.param);

    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","4");
    int iter = 0;
    for(;;)
    {
        const CvMat* _param = 0;
        CvMat* _jac = 0;
        CvMat* _err = 0;

        bool proceed = solver.update(_param, _jac, _err);
        //__android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","4.1");
        cvCopy(_param, &matParams);

        if (!proceed || !_err)
            break;
        //__android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","4.2");
        if (_jac)
        {
            calcJacobian(jac);
            //__android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","4.3");
            CvMat tmp = jac;
            cvCopy(&tmp, _jac);
            //__android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","4.35");
        }

        if (_err)
        {
            calcError(err);
            iter++;
            //__android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","4.4");
            CvMat tmp = err;
            cvCopy(&tmp, _err);
            //__android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","4.45");
        }
        __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","Iter : %d",iter);
    }

    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","5");

    // Check if all camera parameters are valid
    bool ok = true;
    for (int i = 0; i < cam_params_.rows; ++i)
    {
        __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","Param %lf",cam_params_.at<double>(i,0));
        if (cvIsNaN(cam_params_.at<double>(i,0)))
        {
            ok = false;
            break;
        }
    }
    if (!ok)
        return false;

    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","6");
    obtainRefinedCameraParams(cameras);

    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","7");
    Mat R_inv = cameras[0].R.inv();
    //cameras[0].R = R_inv * cameras[0].R;
    //cameras[1].R = R_inv * cameras[1].R;

    // Normalize motion to center image
//    Graph span_tree;
//    std::vector<int> span_tree_centers;
//    findMaxSpanningTree(num_images_, pairwise_matches, span_tree, span_tree_centers);
//    Mat R_inv = cameras[span_tree_centers[0]].R.inv();
//    for (int i = 0; i < num_images_; ++i)
//        cameras[i].R = R_inv * cameras[i].R;

    __android_log_print(ANDROID_LOG_DEBUG,"C++ BundleCV","8");
    return true;
}