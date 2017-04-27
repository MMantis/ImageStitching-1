/*
 * BundleCeres.cpp
 *
 *  Created on: Nov 18, 2558 BE
 *      Author: kunato
 */
#include "BundleCeres.h"
vector<int> point_sizes;
using namespace cv;
using namespace std;

typedef Eigen::Matrix<double,3,3,Eigen::ColMajor> Matrix3d;
typedef Eigen::Matrix<double,3,1,Eigen::ColMajor> Vector3d;

struct ReprojectionErrorData
{
    vector<Point2f> points;
    vector<int> matches_image_idx;
};
struct ReprojectionError {
    ReprojectionError(const double *p1,const double *p2,const double *K1,const double *K2)
            : p1(p1), p2(p2) , K1(K1), K2(K2){}
    bool operator()(const double* rvec1,const double* rvec2,double* residuals) const {
        double R1[9];
        double R2[9];
        Matrix3d eigen_R1;
        Matrix3d eigen_R2;
        Matrix3d eigen_H1;
        Matrix3d eigen_H2;
        Matrix3d eigen_KMat1;
        Matrix3d eigen_KMat2;
        Matrix3d eigen_KInv1;
        Matrix3d eigen_KInv2;
        Vector3d eigen_p1;
        Vector3d eigen_p2;
        Vector3d out1;
        Vector3d out2;
//        cout << x1 << ","<<y1<< " ; " << x2 << "," << y2 << endl;
        eigen_KMat1 << *(K1+0), 0.0, *(K1+2),
                0.0, *(K1+4), *(K1+5),
                0.0,  0.0, 1.0;
        eigen_KMat2 << *(K2+0), 0.0, *(K2+2),
                0.0, *(K2+4), *(K2+5),
                0.0,  0.0, 1.0;
        eigen_p1 << *(p1+0),*(p1+1),*(p1+2);
        eigen_p2 << *(p2+0),*(p2+1),*(p2+2);
        ceres::AngleAxisToRotationMatrix(rvec1,R1);
        ceres::AngleAxisToRotationMatrix(rvec2,R2);
        //order of matrix R
        // [ 0 3 6
        // 	 1 4 7
        //	 2 5 8 ]
        eigen_R1 << R1[0],R1[3],R1[6],R1[1],R1[4],R1[7],R1[2],R1[5],R1[8];
        eigen_R2 << R2[0],R2[3],R2[6],R2[1],R2[4],R2[7],R2[2],R2[5],R2[8];
//        eigen_R1 << R1[0],R1[1],R1[2],R1[3],R1[4],R1[5],R1[6],R1[7],R1[8];
//        eigen_R2 << R2[0],R2[1],R2[2],R2[3],R2[4],R2[5],R2[6],R2[7],R2[8];

        eigen_KInv1 = eigen_KMat1.inverse();
        eigen_KInv2 = eigen_KMat2.inverse();
        //Then find difference between x,y,z
        eigen_H1 = eigen_R1 * eigen_KInv1;
        eigen_H2 = eigen_R2 * eigen_KInv2;

        out1 = eigen_H1*eigen_p1;
        out1/=std::sqrt((out1[0]*out1[0] + out1[1]*out1[1] + out1[2]*out1[2]));
        out2 = eigen_H2*eigen_p2;
        out2/std::sqrt((out2[0]*out2[0] + out2[1]*out2[1] + out2[2]*out2[2]));
        out1-=out2;
        cout << out1 << endl;
        residuals[0] = out1[0];
        residuals[1] = out1[1];
        residuals[2] = out1[2];
        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(
            const double *p1,
            const double *p2,
            const double *K1,
            const double *K2) {
        return (new ceres::NumericDiffCostFunction<ReprojectionError,ceres::CENTRAL, 3, 3, 3>(new ReprojectionError(p1,p2,K1,K2)));
    }

    const double *p1;
    const double *p2;
    const double *K1;
    const double *K2;
};


struct BaseErrorOneVector {
    BaseErrorOneVector(const double *rvec1,const double *o_rvec2,const int num_point)
            : rvec1(rvec1), o_rvec2(o_rvec2), num_point(num_point){}
    bool operator()(const double* rvec2,double* residuals) const {
        double R1[9];
        double R2[9];
        double O_R2[9];
        Matrix3d diff_modify;
        Matrix3d diff_original;
        ceres::AngleAxisToRotationMatrix(rvec1,R1);
        ceres::AngleAxisToRotationMatrix(rvec2,R2);
        ceres::AngleAxisToRotationMatrix(o_rvec2,O_R2);
        Matrix3d eigen_R1(R1);
        Matrix3d eigen_R2(R2);
        Matrix3d eigen_OR2(O_R2);
        diff_modify = eigen_R2 * eigen_R1.inverse();
        diff_original = eigen_OR2 * eigen_R1.inverse();

        //maybe this is enough

        Matrix3d diff;
        diff = eigen_OR2 * eigen_R2.inverse();
        double diff_quat[4];
        //calculate diff of 3 axis
        double *diff_array = diff.data();
        ceres::RotationMatrixToQuaternion(diff_array,diff_quat);
        double angle = 2 * asin(diff_quat[3]);
//        __android_log_print(ANDROID_LOG_DEBUG,"Ceres Minimize","Diff Angle [%lf]",angle*180.0/M_PI);

        double multiplier = 200 - (200 * (log(num_point)/log(50)));
        if(multiplier > 0){
            multiplier = multiplier;
        }
        else{
            multiplier = 0;
        }
        //__android_log_print(ANDROID_LOG_DEBUG,"Ceres Minimize","Point Size : %d : %lf",num_point,multiplier);
        residuals[0] = multiplier * abs(angle);
//        residuals[0] = 0;

//        __android_log_print(ANDROID_LOG_DEBUG,"Ceres Minimize","Bases Residuals : %lf",residuals[0]);
        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(
            const double *rvec1,
            const double *o_rvec2,
            const int num_point) {
        return (new ceres::NumericDiffCostFunction<BaseErrorOneVector,ceres::CENTRAL, 1, 3>(new BaseErrorOneVector(rvec1,o_rvec2,num_point)));
    }
    const int num_point;
    const double *rvec1;
    const double *o_rvec2;
};



struct ReprojectionErrorOneVector {
    ReprojectionErrorOneVector(const double *p1,const double *p2,const double *K1,const double *K2,const double *rvec1)
            : p1(p1), p2(p2) , K1(K1), K2(K2), rvec1(rvec1){}
    bool operator()(const double* rvec2,double* residuals) const {
        typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dr;
        double R1[9];
        double R2[9];
        Matrix3d eigen_H1;
        Matrix3d eigen_H2;
        Matrix3dr eigen_KMat1(K1);
        Matrix3dr eigen_KMat2(K2);
        Matrix3d eigen_KInv1;
        Matrix3d eigen_KInv2;
        Vector3d out1;
        Vector3d out2;

        Vector3d eigen_p1(p1);
        Vector3d eigen_p2(p2);
        ceres::AngleAxisToRotationMatrix(rvec1,R1);
        ceres::AngleAxisToRotationMatrix(rvec2,R2);

        Matrix3d eigen_R1(R1);
        Matrix3d eigen_R2(R2);
        //order of matrix R
        // [ 0 3 6
        // 	 1 4 7
        //	 2 5 8 ]
//        eigen_R1 << R1[0],R1[3],R1[6],R1[1],R1[4],R1[7],R1[2],R1[5],R1[8];
//        eigen_R2 << R2[0],R2[3],R2[6],R2[1],R2[4],R2[7],R2[2],R2[5],R2[8];
        eigen_KInv1 = eigen_KMat1.inverse();
        eigen_KInv2 = eigen_KMat2.inverse();
        //Then find difference between x,y,z
        eigen_H1 = eigen_R1 * eigen_KInv1;
        eigen_H2 = eigen_R2 * eigen_KInv2;

        out1 = eigen_H1*eigen_p1;
        out1/=std::sqrt((out1[0]*out1[0] + out1[1]*out1[1] + out1[2]*out1[2]));
        out2 = eigen_H2*eigen_p2;
        out2/std::sqrt((out2[0]*out2[0] + out2[1]*out2[1] + out2[2]*out2[2]));
        out1-=out2;
        residuals[0] = std::abs(out1[0])+std::abs(out1[1])+std::abs(out1[2]);
//        __android_log_print(ANDROID_LOG_DEBUG,"Ceres Minimize","Additional Residuals : %lf",residuals[0]);

        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(
            const double *p1,
            const double *p2,
            const double *K1,
            const double *K2,
            const double *rvec1) {
        return (new ceres::NumericDiffCostFunction<ReprojectionErrorOneVector,ceres::CENTRAL, 1, 3>(new ReprojectionErrorOneVector(p1,p2,K1,K2,rvec1)));
    }

    const double *p1;
    const double *p2;
    const double *K1;
    const double *K2;
    const double *rvec1;
};




//Global Minimize
int minimizeRotation(vector<ImageFeatures> features,vector<MatchesInfo> pairs,vector<CameraParams> &cameras){

    vector<ReprojectionErrorData> rpSet;
    cout << "point set" << endl;
    for(int i = 0 ; i < pairs.size() ;i++){
        if(pairs[i].confidence < 0.9)
            continue;
        for(int j = 0 ; j < pairs[i].inliers_mask.size() ; j++){
            if(!pairs[i].inliers_mask[j]){
                continue;
            }
            vector<Point2f> pointSet;
            vector<int> image_idx;
            pointSet.push_back(features[pairs[i].src_img_idx].keypoints[pairs[i].matches[j].queryIdx].pt);
            image_idx.push_back(pairs[i].src_img_idx);
            pointSet.push_back(features[pairs[i].dst_img_idx].keypoints[pairs[i].matches[j].trainIdx].pt);
            image_idx.push_back(pairs[i].dst_img_idx);
            ReprojectionErrorData rpData;
            rpData.points = pointSet;
            rpData.matches_image_idx = image_idx;
            rpSet.push_back(rpData);
        }
    }
    cout << "Ending add rpSet" <<endl;
    ceres::Problem problem;
    double focal_array[cameras.size()];
    double *focal_pointer = focal_array;
    double rotation_array[cameras.size()*3];
    double *rotation_pointer = rotation_array;
    double K_array[cameras.size()*9];
    double *K_pointer = K_array;
    for(int i = 0 ; i < cameras.size() ; i++){
        focal_array[i] = cameras[i].focal;
        Mat R = cameras[i].R;
        Mat rvec;
        Rodrigues(R,rvec);
        for(int j = 0 ; j < 3 ; j++){
            rotation_array[i*3+j] = rvec.at<float>(j);
        }
        Mat_<double> K = cameras[i].K();
        for(int j = 0 ; j < 9 ; j ++){
            K_array[i*9+j] = K.at<double>(j);
        }
    }
    cout << "Ending create rotation array" << endl;
    cout << "Ending create K Matrix" << endl;
    double p1[rpSet.size()*3];
    double p2[rpSet.size()*3];
    for(int i = 0 ; i < rpSet.size();i++){
        p1[i*3] = rpSet[i].points[0].x;
        p1[i*3+1] = rpSet[i].points[0].y;
        p1[i*3+2] = 1;
        p2[i*3] = rpSet[i].points[1].x;
        p2[i*3+1] = rpSet[i].points[1].y;
        p2[i*3+2] = 1;
    }
    for(int i = rpSet.size() -1 ; i >= 0  ; i--){
        double *p1_pointer = (p1+(i*3));
        double *p2_pointer = (p2+(i*3));

        ceres::CostFunction* cost_func = ReprojectionError::Create(p1_pointer,p2_pointer,
                                                                   K_pointer + rpSet[i].matches_image_idx[0]*9,
                                                                   K_pointer + rpSet[i].matches_image_idx[1]*9);
        problem.AddResidualBlock(cost_func,NULL, rotation_pointer + rpSet[i].matches_image_idx[0]*3
                , rotation_pointer + rpSet[i].matches_image_idx[1]*3);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    int iterationCount = summary.iterations.size();
    __android_log_print(ANDROID_LOG_DEBUG,"Ceres","Iteration %d",iterationCount);
    cv::FileStorage rfs("/sdcard/stitch/rotation2.yml", cv::FileStorage::WRITE);
    for(int i = 0 ; i < cameras.size() ; i ++){
        Mat R;
        Mat rvec = Mat::zeros(3,1,CV_64F);
        for(int j = 0 ; j < 3 ; j++){
            rvec.at<double>(j) = rotation_array[i*3+j];
        }
        Rodrigues(rvec,R);
        rfs << "i" << R;
        R.convertTo(cameras[i].R,CV_32F);
    }
    Mat R_inv = cameras[0].R.inv();
    for(int i = 0 ; i < cameras.size() ;i++){
        cameras[i].R = R_inv * cameras[i].R;
    }
    return iterationCount;

}



//Local Minimize
int minimizeRotation(vector<Point2f> src,vector<Point2f> dst,vector<CameraParams> &cameras){

    vector<ReprojectionErrorData> rpSet;
    cout << "point set" << endl;

        for(int j = 0 ; j < src.size() ; j++){
            vector<Point2f> pointSet;
            vector<int> image_idx;
            pointSet.push_back(src[j]);
            image_idx.push_back(0);
            pointSet.push_back(dst[j]);
            image_idx.push_back(1);
            ReprojectionErrorData rpData;
            rpData.points = pointSet;
            rpData.matches_image_idx = image_idx;
            rpSet.push_back(rpData);
        }
    cout << "Ending add rpSet" <<endl;
    ceres::Problem problem;
    double focal_array[cameras.size()];
    double *focal_pointer = focal_array;
    double rotation_array[cameras.size()*3];
    double *rotation_pointer = rotation_array;
    double K_array[cameras.size()*9];
    double *K_pointer = K_array;
    for(int i = 0 ; i < cameras.size() ; i++){
        focal_array[i] = cameras[i].focal;
        Mat R = cameras[i].R;
        Mat rvec;
        Rodrigues(R,rvec);
        for(int j = 0 ; j < 3 ; j++){
            rotation_array[i*3+j] = rvec.at<float>(j);
        }
        Mat_<double> K = cameras[i].K();
        for(int j = 0 ; j < 9 ; j ++){
            K_array[i*9+j] = K.at<double>(j);
        }
    }
    cout << "Ending create rotation array" << endl;
    cout << "Ending create K Matrix" << endl;
    double p1[rpSet.size()*3];
    double p2[rpSet.size()*3];
    for(int i = 0 ; i < rpSet.size();i++){
        p1[i*3] = rpSet[i].points[0].x;
        p1[i*3+1] = rpSet[i].points[0].y;
        p1[i*3+2] = 1;
        p2[i*3] = rpSet[i].points[1].x;
        p2[i*3+1] = rpSet[i].points[1].y;
        p2[i*3+2] = 1;
    }
    for(int i = rpSet.size() -1 ; i >= 0  ; i--){
        double *p1_pointer = (p1+(i*3));
        double *p2_pointer = (p2+(i*3));

        ceres::CostFunction* cost_func = ReprojectionErrorOneVector::Create(p1_pointer,p2_pointer,
                                                                   K_pointer + rpSet[i].matches_image_idx[0]*9,
                                                                   K_pointer + rpSet[i].matches_image_idx[1]*9,
                                                                   rotation_pointer + rpSet[i].matches_image_idx[0]*3);
        problem.AddResidualBlock(cost_func,NULL,
                rotation_pointer + rpSet[i].matches_image_idx[1]*3);
    }
    ceres::CostFunction* cost_func = BaseErrorOneVector::Create(rotation_pointer + rpSet[0].matches_image_idx[0]*3,rotation_pointer + rpSet[0].matches_image_idx[1]*3,rpSet.size());
    problem.AddResidualBlock(cost_func,NULL, rotation_pointer + rpSet[0].matches_image_idx[1]*3);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    int iterationCount = summary.iterations.size();
    __android_log_print(ANDROID_LOG_DEBUG,"Ceres","Iteration %d",iterationCount);
    for(int i = 0 ; i < cameras.size() ; i ++){
        Mat R;
        Mat rvec = Mat::zeros(3,1,CV_64F);
        for(int j = 0 ; j < 3 ; j++){
            rvec.at<double>(j) = rotation_array[i*3+j];
        }
        Rodrigues(rvec,R);
        R.convertTo(cameras[i].R,CV_32F);
    }
    point_sizes.push_back(rpSet.size());
    ofstream myfile;
    myfile.open ("/sdcard/stitch/point.txt");
    for(int i = 0 ; i < point_sizes.size() ; i++){

        myfile << i<<" : "<<point_sizes[i] <<"\n";

    }
    myfile.close();
    return iterationCount;
}
