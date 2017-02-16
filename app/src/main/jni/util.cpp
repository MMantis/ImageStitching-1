//
// Created by Kunat Pipatanakul on 3/31/16.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include "util.h"
using namespace cv::detail;

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

void calcDeriv(const Mat &err1, const Mat &err2, double h, Mat res)
{
    for (int i = 0; i < err1.rows; ++i)
        res.at<double>(i, 0) = (err2.at<double>(i, 0) - err1.at<double>(i, 0)) / h;
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

static void
getRTMatrix( const Point2f* a, const Point2f* b,
             int count, Mat& M, bool fullAffine )
{
    CV_Assert( M.isContinuous() );

    if( fullAffine )
    {
        double sa[6][6]={{0.}}, sb[6]={0.};
        Mat A( 6, 6, CV_64F, &sa[0][0] ), B( 6, 1, CV_64F, sb );
        Mat MM = M.reshape(1, 6);

        for( int i = 0; i < count; i++ )
        {
            sa[0][0] += a[i].x*a[i].x;
            sa[0][1] += a[i].y*a[i].x;
            sa[0][2] += a[i].x;

            sa[1][1] += a[i].y*a[i].y;
            sa[1][2] += a[i].y;

            sb[0] += a[i].x*b[i].x;
            sb[1] += a[i].y*b[i].x;
            sb[2] += b[i].x;
            sb[3] += a[i].x*b[i].y;
            sb[4] += a[i].y*b[i].y;
            sb[5] += b[i].y;
        }

        sa[3][4] = sa[4][3] = sa[1][0] = sa[0][1];
        sa[3][5] = sa[5][3] = sa[2][0] = sa[0][2];
        sa[4][5] = sa[5][4] = sa[2][1] = sa[1][2];

        sa[3][3] = sa[0][0];
        sa[4][4] = sa[1][1];
        sa[5][5] = sa[2][2] = count;

        solve( A, B, MM, DECOMP_EIG );
    }
    else
    {
        double sa[4][4]={{0.}}, sb[4]={0.}, m[4];
        Mat A( 4, 4, CV_64F, sa ), B( 4, 1, CV_64F, sb );
        Mat MM( 4, 1, CV_64F, m );

        for( int i = 0; i < count; i++ )
        {
            sa[0][0] += a[i].x*a[i].x + a[i].y*a[i].y;
            sa[0][2] += a[i].x;
            sa[0][3] += a[i].y;

            sb[0] += a[i].x*b[i].x + a[i].y*b[i].y;
            sb[1] += a[i].x*b[i].y - a[i].y*b[i].x;
            sb[2] += b[i].x;
            sb[3] += b[i].y;
        }

        sa[1][1] = sa[0][0];
        sa[2][1] = sa[1][2] = -sa[0][3];
        sa[3][1] = sa[1][3] = sa[2][0] = sa[0][2];
        sa[2][2] = sa[3][3] = count;
        sa[3][0] = sa[0][3];

        solve( A, B, MM, DECOMP_EIG );

        double* om = M.ptr<double>();
        om[0] = om[4] = m[0];
        om[1] = -m[1];
        om[3] = m[1];
        om[2] = m[2];
        om[5] = m[3];
    }
}

cv::Mat estimateRigidTransform( InputArray src1, InputArray src2, bool fullAffine ,vector<uchar> &ransac )
{
    __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","Modified RANSAC");
    Mat M(2, 3, CV_64F), A = src1.getMat(), B = src2.getMat();

    const int COUNT = 15;
    const int WIDTH = 160, HEIGHT = 120;
    const int RANSAC_MAX_ITERS = 50000;
    const int RANSAC_SIZE0 = 3;
    const double RANSAC_GOOD_RATIO = 0.5;

    std::vector<Point2f> pA, pB;
    std::vector<int> good_idx;
    std::vector<uchar> status;

    double scale = 1.;
    int i, j, k, k1;

    RNG rng((uint64)-1);
    int good_count = 0;

    if( A.size() != B.size() )
        CV_Error( Error::StsUnmatchedSizes, "Both input images must have the same size" );

    if( A.type() != B.type() )
        CV_Error( Error::StsUnmatchedFormats, "Both input images must have the same data type" );

    int count = A.checkVector(2);

    if( count > 0 )
    {
        A.reshape(2, count).convertTo(pA, CV_32F);
        B.reshape(2, count).convertTo(pB, CV_32F);
    }
    else if( A.depth() == CV_8U )
    {
        int cn = A.channels();
        CV_Assert( cn == 1 || cn == 3 || cn == 4 );
        Size sz0 = A.size();
        Size sz1(WIDTH, HEIGHT);

        scale = std::max(1., std::max( (double)sz1.width/sz0.width, (double)sz1.height/sz0.height ));

        sz1.width = cvRound( sz0.width * scale );
        sz1.height = cvRound( sz0.height * scale );

        bool equalSizes = sz1.width == sz0.width && sz1.height == sz0.height;

        if( !equalSizes || cn != 1 )
        {
            Mat sA, sB;

            if( cn != 1 )
            {
                Mat gray;
                cvtColor(A, gray, COLOR_BGR2GRAY);
                resize(gray, sA, sz1, 0., 0., INTER_AREA);
                cvtColor(B, gray, COLOR_BGR2GRAY);
                resize(gray, sB, sz1, 0., 0., INTER_AREA);
            }
            else
            {
                resize(A, sA, sz1, 0., 0., INTER_AREA);
                resize(B, sB, sz1, 0., 0., INTER_AREA);
            }

            A = sA;
            B = sB;
        }

        int count_y = COUNT;
        int count_x = cvRound((double)COUNT*sz1.width/sz1.height);
        count = count_x * count_y;

        pA.resize(count);
        pB.resize(count);
        status.resize(count);

        for( i = 0, k = 0; i < count_y; i++ )
            for( j = 0; j < count_x; j++, k++ )
            {
                pA[k].x = (j+0.5f)*sz1.width/count_x;
                pA[k].y = (i+0.5f)*sz1.height/count_y;
            }

        // find the corresponding points in B
        calcOpticalFlowPyrLK(A, B, pA, pB, status, noArray(), Size(21, 21), 3,
                             TermCriteria(TermCriteria::MAX_ITER,40,0.1));

        // repack the remained points
        for( i = 0, k = 0; i < count; i++ )
            if( status[i] )
            {
                if( i > k )
                {
                    pA[k] = pA[i];
                    pB[k] = pB[i];
                }
                k++;
            }
        count = k;
        pA.resize(count);
        pB.resize(count);
    }
    else
        CV_Error( Error::StsUnsupportedFormat, "Both input images must have either 8uC1 or 8uC3 type" );

    good_idx.resize(count);

    if( count < RANSAC_SIZE0 )
        return Mat();

    Rect brect = boundingRect(pB);

    // RANSAC stuff:
    // 1. find the consensus
    for( k = 0; k < RANSAC_MAX_ITERS; k++ )
    {
        int idx[RANSAC_SIZE0];
        Point2f a[RANSAC_SIZE0];
        Point2f b[RANSAC_SIZE0];

        // choose random 3 non-complanar points from A & B
        for( i = 0; i < RANSAC_SIZE0; i++ )
        {
            for( k1 = 0; k1 < RANSAC_MAX_ITERS; k1++ )
            {
                idx[i] = rng.uniform(0, count);

                for( j = 0; j < i; j++ )
                {
                    if( idx[j] == idx[i] )
                        break;
                    // check that the points are not very close one each other
                    if( fabs(pA[idx[i]].x - pA[idx[j]].x) +
                        fabs(pA[idx[i]].y - pA[idx[j]].y) < FLT_EPSILON )
                        break;
                    if( fabs(pB[idx[i]].x - pB[idx[j]].x) +
                        fabs(pB[idx[i]].y - pB[idx[j]].y) < FLT_EPSILON )
                        break;
                }

                if( j < i )
                    continue;

                if( i+1 == RANSAC_SIZE0 )
                {
                    // additional check for non-complanar vectors
                    a[0] = pA[idx[0]];
                    a[1] = pA[idx[1]];
                    a[2] = pA[idx[2]];

                    b[0] = pB[idx[0]];
                    b[1] = pB[idx[1]];
                    b[2] = pB[idx[2]];

                    double dax1 = a[1].x - a[0].x, day1 = a[1].y - a[0].y;
                    double dax2 = a[2].x - a[0].x, day2 = a[2].y - a[0].y;
                    double dbx1 = b[1].x - b[0].x, dby1 = b[1].y - b[0].y;
                    double dbx2 = b[2].x - b[0].x, dby2 = b[2].y - b[0].y;
                    const double eps = 0.01;

                    if( fabs(dax1*day2 - day1*dax2) < eps*std::sqrt(dax1*dax1+day1*day1)*std::sqrt(dax2*dax2+day2*day2) ||
                        fabs(dbx1*dby2 - dby1*dbx2) < eps*std::sqrt(dbx1*dbx1+dby1*dby1)*std::sqrt(dbx2*dbx2+dby2*dby2) )
                        continue;
                }
                break;
            }

            if( k1 >= RANSAC_MAX_ITERS )
                break;
        }

        if( i < RANSAC_SIZE0 )
            continue;

        // estimate the transformation using 3 points
        getRTMatrix( a, b, 3, M, fullAffine );

        const double* m = M.ptr<double>();
        for( i = 0, good_count = 0; i < count; i++ )
        {
            if( std::abs( m[0]*pA[i].x + m[1]*pA[i].y + m[2] - pB[i].x ) +
                std::abs( m[3]*pA[i].x + m[4]*pA[i].y + m[5] - pB[i].y ) < std::max(brect.width,brect.height)*0.05 )
                good_idx[good_count++] = i;
        }

        if( good_count >= count*RANSAC_GOOD_RATIO )
            break;
    }

    if( k >= RANSAC_MAX_ITERS ) {
        __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","RANSAC FAILED");
        return Mat();
    }
    __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","RANSAC Good Size %d from %d",good_count,count);
    if( good_count <= count )
    {
        __android_log_print(ANDROID_LOG_DEBUG,"C++ Tracking","RANSAC Assigned");
        ransac.assign(src1.total(),0);
        for( i = 0; i < good_count; i++ )
        {
            j = good_idx[i];
            pA[i] = pA[j];
            pB[i] = pB[j];
            ransac[j] = 1;
        }
    }

    getRTMatrix( &pA[0], &pB[0], good_count, M, fullAffine );
    M.at<double>(0, 2) /= scale;
    M.at<double>(1, 2) /= scale;


    return M;
}







