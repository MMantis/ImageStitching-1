//
// Created by Kunat Pipatanakul on 3/31/16.
//

#include "util.h"
Mat multiply(Mat a,Mat b){
    int m = a.cols;
    int y = b.cols;
    int x = b.rows;
    Mat out(m,y,a.type());
    for(int i = 0; i < y ; i++) {
        for (int j = 0; j < x ; j++) {
            float temp = 0;
            for (int k = 0; k < m; k++) {
                temp = temp + (a.at<float>(i, k) * b.at<float>(k, j));
            }
            out.at<float>(i, j) = temp;
        }
    }
    return out;
}

//TODO Fix sigmentation fault
void remap(Mat src,Mat &dst,Mat xmap,Mat ymap){
    dst = Mat(xmap.size(),CV_8UC3);
    __android_log_print(ANDROID_LOG_INFO,"C++ Remap","First %d %d %d %d",ymap.rows,ymap.cols,xmap.rows,xmap.cols);
    for(int i = 0 ; i < xmap.rows;i++){
        for(int j = 0 ; j < xmap.cols; j++){

            //dst.at<Vec3b>(i,j) = src.at<Vec3b>(ymap.at<float>(i,j),xmap.at<float>(i,j));
            float y_coord = ymap.at<float>(i,j);
            float x_coord = xmap.at<float>(i,j);
            x_coord = (x_coord < 0) ? 0 : x_coord;
            x_coord = (x_coord >= src.cols) ? src.cols-1 : x_coord;

            y_coord = (y_coord < 0) ? 0 : y_coord;
            y_coord = (y_coord >= src.rows) ? src.rows-1 : y_coord;
            dst.at<Vec3b>(i,j) = src.at<Vec3b>(y_coord, x_coord);
        }
    }
}


