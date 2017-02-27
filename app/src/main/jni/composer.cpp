#include "composer.h"
using namespace std;
using namespace cv;
using namespace cv::detail;
namespace composer {
    static const float WEIGHT_EPS = 1e-5f;
#define BLENDING
#define ALPHA

#ifndef BLENDING
    Mat dst_, dst_mask_,dst_dt_;
    Rect dst_roi_;
    void prepare(Rect dst_roi,int not_used){
        __android_log_print(ANDROID_LOG_DEBUG,"Composer","Prepare Normal");
        dst_.create(dst_roi.size(), CV_8UC3);
        dst_.setTo(Scalar::all(0));
        dst_mask_.create(dst_roi.size(), CV_8U);
        dst_mask_.setTo(Scalar::all(0));
        dst_roi_ = dst_roi;
        dst_dt_.create(dst_roi.size(),CV_8U);
        __android_log_print(ANDROID_LOG_DEBUG,"Composer","Prepared");
    }

    void feed(Mat img,Mat mask,Point tl){
        clock_t c_before = std::clock();
        CV_Assert(img.type() == CV_8UC3);
        CV_Assert(mask.type() == CV_8U);
        int dx = tl.x - dst_roi_.x;
        int dy = tl.y - dst_roi_.y;

        for (int y = 0; y < img.rows; ++y)
        {
            const Vec3b *src_row = img.ptr<Vec3b>(y);
            Vec3b *dst_row = dst_.ptr<Vec3b>(dy + y);
            const uchar *mask_row = mask.ptr<uchar>(y);
            uchar *dst_mask_row = dst_mask_.ptr<uchar>(dy + y);

            for (int x = 0; x < img.cols; ++x)
            {
                if (mask_row[x]) {
                    dst_row[dx + x] = src_row[x];
                    dst_mask_row[dx + x] = 1;
                }

            }
        }
        clock_t c_after = std::clock();
        __android_log_print(ANDROID_LOG_DEBUG,"C++ Composer","Feed %lf",(double)(c_after-c_before)/CLOCKS_PER_SEC );
    }

    void process(Mat &dst,Mat &dst_mask){
        __android_log_print(ANDROID_LOG_DEBUG,"Composer","Process");
        dst_.setTo(Scalar::all(0), dst_mask_ == 0);
        int channels_setting[] = {2,0, 1,1, 0,2, 3,3};
        dst.create(dst_.size(),CV_8UC4);

        int max_x = dst_roi_.size().height;
        int max_y = dst_roi_.size().width;
        for(int x = 0 ; x < dst_roi_.size().height; x++){
            for(int y = 0 ; y < dst_roi_.size().width ; y++){
//                calc distance transform (replace 0.1)
                //large = avoid seam small = avoid ghost
                float window_size = 200.0;
                float dist_x = 0;
                float dist_y = 0;
                if(x < max_x-x){
                    dist_x = x/window_size;
                }
                else{
                    dist_x = (max_x-x)/window_size;
                }
                if(y < max_y-y){
                    dist_y = (y)/window_size;
                }
                else{
                    dist_y = (max_y-y)/window_size;
                }
                if(dist_x > 1.0)
                    dist_x = 1.0;
                if(dist_y > 1.0)
                    dist_y = 1.0;
                if(dist_x > dist_y){
                    dst_dt_.at<uchar>(x,y) = 255;
                }
                else{
                    dst_dt_.at<uchar>(x,y) = 255;
                }
            }
        }
        Mat temp = dst_mask_.mul(dst_dt_);

        Mat out[] = {dst_,temp};
        mixChannels(out,2,&dst,1,channels_setting,4);
//        dst = dst_;

        dst_mask = temp;
        dst_dt_.release();
        dst_.release();
        dst_mask_.release();
    }

#endif

#ifdef PYRAMID
    //
    //  $$$$$$$         $$$   $$$           $$$$$
    //  $$               $$$ $$$            $$$ $$
    //  $$$$$$$            $$$              $$$$$
    //  $$               $$$ $$$            $$$
    //  $$$$$$$         $$$   $$$           $$$


    // Pyramid Blending


    Mat dst_, dst_mask_,dst_dt_;
    std::vector<Mat> dst_pyr_laplace_;
    std::vector<Mat> dst_band_weights_;
    Rect dst_roi_,dst_roi_final_;
    int num_bands_,actual_num_bands_;
    int weight_type_ = CV_16S;
    void prepare(Rect dst_roi,int num_bands){
        __android_log_print(ANDROID_LOG_DEBUG,"Composer","Prepare Pyramid");
        dst_roi_final_ = dst_roi;
        actual_num_bands_ = num_bands;
        // Crop unnecessary bands
        double max_len = static_cast<double>(max(dst_roi.width, dst_roi.height));
        num_bands_ = min(actual_num_bands_, static_cast<int>(ceil(log(max_len) / log(2.0))));
        __android_log_print(ANDROID_LOG_DEBUG,"Composer","Pyramid %d band",num_bands_);
        // Add border to the final image, to ensure sizes are divided by (1 << num_bands_)
        dst_roi.width += ((1 << num_bands_) - dst_roi.width % (1 << num_bands_)) % (1 << num_bands_);
        dst_roi.height += ((1 << num_bands_) - dst_roi.height % (1 << num_bands_)) % (1 << num_bands_);

        dst_.create(dst_roi.size(), CV_16SC3);
        dst_.setTo(Scalar::all(0));
        dst_mask_.create(dst_roi.size(), CV_8U);
        dst_mask_.setTo(Scalar::all(0));
        dst_roi_ = dst_roi;
        dst_dt_.create(dst_roi.size(),CV_8U);


        dst_pyr_laplace_.resize(num_bands_ + 1);
        dst_pyr_laplace_[0] = dst_;

        dst_band_weights_.resize(num_bands_ + 1);
        dst_band_weights_[0].create(dst_roi.size(), weight_type_);
        dst_band_weights_[0].setTo(0);

        for (int i = 1; i <= num_bands_; ++i)
        {
            dst_pyr_laplace_[i].create((dst_pyr_laplace_[i - 1].rows + 1) / 2,
                                       (dst_pyr_laplace_[i - 1].cols + 1) / 2, CV_16SC3);
            dst_band_weights_[i].create((dst_band_weights_[i - 1].rows + 1) / 2,
                                        (dst_band_weights_[i - 1].cols + 1) / 2, weight_type_);
            dst_pyr_laplace_[i].setTo(Scalar::all(0));
            dst_band_weights_[i].setTo(0);
        }
        __android_log_print(ANDROID_LOG_DEBUG,"Composer","Prepared Pyramid");
    }

    void feed(const Mat input_img,const Mat mask, Point tl)
    {
        Mat img;
        input_img.convertTo(img,CV_16SC3);

        CV_Assert(img.type() == CV_16SC3 || img.type() == CV_8UC3);
        CV_Assert(mask.type() == CV_8U);

        // Keep source image in memory with small border
        int gap = 3 * (1 << num_bands_);
        Point tl_new(max(dst_roi_.x, tl.x - gap),
                     max(dst_roi_.y, tl.y - gap));
        Point br_new(min(dst_roi_.br().x, tl.x + img.cols + gap),
                     min(dst_roi_.br().y, tl.y + img.rows + gap));

        // Ensure coordinates of top-left, bottom-right corners are divided by (1 << num_bands_).
        // After that scale between layers is exactly 2.
        //
        // We do it to avoid interpolation problems when keeping sub-images only. There is no such problem when
        // image is bordered to have size equal to the final image size, but this is too memory hungry approach.
        tl_new.x = dst_roi_.x + (((tl_new.x - dst_roi_.x) >> num_bands_) << num_bands_);
        tl_new.y = dst_roi_.y + (((tl_new.y - dst_roi_.y) >> num_bands_) << num_bands_);
        int width = br_new.x - tl_new.x;
        int height = br_new.y - tl_new.y;
        width += ((1 << num_bands_) - width % (1 << num_bands_)) % (1 << num_bands_);
        height += ((1 << num_bands_) - height % (1 << num_bands_)) % (1 << num_bands_);
        br_new.x = tl_new.x + width;
        br_new.y = tl_new.y + height;
        int dy = max(br_new.y - dst_roi_.br().y, 0);
        int dx = max(br_new.x - dst_roi_.br().x, 0);
        tl_new.x -= dx; br_new.x -= dx;
        tl_new.y -= dy; br_new.y -= dy;

        int top = tl.y - tl_new.y;
        int left = tl.x - tl_new.x;
        int bottom = br_new.y - tl.y - img.rows;
        int right = br_new.x - tl.x - img.cols;

        // Create the source image Laplacian pyramid
        Mat img_with_border;
        copyMakeBorder(img, img_with_border, top, bottom, left, right,
                       BORDER_REFLECT);
        vector<Mat> src_pyr_laplace;

        createLaplacePyr(img_with_border, num_bands_, src_pyr_laplace);

        // Create the weight map Gaussian pyramid
        Mat weight_map;
        vector<Mat> weight_pyr_gauss(num_bands_ + 1);

        if(weight_type_ == CV_32F)
        {
            mask.convertTo(weight_map, CV_32F, 1./255.);
        }
        else// weight_type_ == CV_16S
        {
            mask.convertTo(weight_map, CV_16S);
            add(weight_map, 1, weight_map, mask != 0);
        }

        copyMakeBorder(weight_map, weight_pyr_gauss[0], top, bottom, left, right, BORDER_CONSTANT);

        for (int i = 0; i < num_bands_; ++i)
            pyrDown(weight_pyr_gauss[i], weight_pyr_gauss[i + 1]);

        int y_tl = tl_new.y - dst_roi_.y;
        int y_br = br_new.y - dst_roi_.y;
        int x_tl = tl_new.x - dst_roi_.x;
        int x_br = br_new.x - dst_roi_.x;

        // Add weighted layer of the source image to the final Laplacian pyramid layer
        if(weight_type_ == CV_32F)
        {
            for (int i = 0; i <= num_bands_; ++i)
            {
                for (int y = y_tl; y < y_br; ++y)
                {
                    int y_ = y - y_tl;
                    const Point3_<short>* src_row = src_pyr_laplace[i].ptr<Point3_<short> >(y_);
                    Point3_<short>* dst_row = dst_pyr_laplace_[i].ptr<Point3_<short> >(y);
                    const float* weight_row = weight_pyr_gauss[i].ptr<float>(y_);
                    float* dst_weight_row = dst_band_weights_[i].ptr<float>(y);

                    for (int x = x_tl; x < x_br; ++x)
                    {
                        int x_ = x - x_tl;
                        dst_row[x].x += static_cast<short>(src_row[x_].x * weight_row[x_]);
                        dst_row[x].y += static_cast<short>(src_row[x_].y * weight_row[x_]);
                        dst_row[x].z += static_cast<short>(src_row[x_].z * weight_row[x_]);
                        dst_weight_row[x] += weight_row[x_];
                    }
                }
                x_tl /= 2; y_tl /= 2;
                x_br /= 2; y_br /= 2;
            }
        }
        else// weight_type_ == CV_16S
        {
            for (int i = 0; i <= num_bands_; ++i)
            {
                for (int y = y_tl; y < y_br; ++y)
                {
                    int y_ = y - y_tl;
                    const Point3_<short>* src_row = src_pyr_laplace[i].ptr<Point3_<short> >(y_);
                    Point3_<short>* dst_row = dst_pyr_laplace_[i].ptr<Point3_<short> >(y);
                    const short* weight_row = weight_pyr_gauss[i].ptr<short>(y_);
                    short* dst_weight_row = dst_band_weights_[i].ptr<short>(y);

                    for (int x = x_tl; x < x_br; ++x)
                    {
                        int x_ = x - x_tl;
                        dst_row[x].x += short((src_row[x_].x * weight_row[x_]) >> 8);
                        dst_row[x].y += short((src_row[x_].y * weight_row[x_]) >> 8);
                        dst_row[x].z += short((src_row[x_].z * weight_row[x_]) >> 8);
                        dst_weight_row[x] += weight_row[x_];
                    }
                }
                x_tl /= 2; y_tl /= 2;
                x_br /= 2; y_br /= 2;
            }
        }
    }


    //TODO Improved
    void process(Mat &dst,Mat &dst_mask){
        for (int i = 0; i <= num_bands_; ++i)
            normalizeUsingWeightMap(dst_band_weights_[i], dst_pyr_laplace_[i]);
        restoreImageFromLaplacePyr(dst_pyr_laplace_);
        dst_ = dst_pyr_laplace_[0];
        dst_ = dst_(Range(0, dst_roi_final_.height), Range(0, dst_roi_final_.width));

        compare(dst_band_weights_[0], WEIGHT_EPS, dst_mask_, CMP_GT);


        dst_mask_ = dst_mask_(Range(0, dst_roi_final_.height), Range(0, dst_roi_final_.width));
        imwrite("/sdcard/stitch/band_weight.jpg",dst_mask_);
        dst_pyr_laplace_.clear();
        dst_band_weights_.clear();

        __android_log_print(ANDROID_LOG_DEBUG,"C++ Composer","Process");

        dst_.setTo(Scalar::all(0), dst_mask_ == 0 );


        dst.create(dst_.size(),CV_8UC4);

        int max_x = dst_roi_final_.size().height;
        int max_y = dst_roi_final_.size().width;
        for(int x = 0 ; x < dst_roi_.size().height; x++){
            for(int y = 0 ; y < dst_roi_.size().width ; y++){
                //                calc distance transform (replace 0.1)
                //large = avoid seam small = avoid ghost
                float window_size = 100.0;
                float dist_x = 0;
                float dist_y = 0;
                if(x < max_x-x){
                    dist_x = x/window_size;
                }
                else{
                    dist_x = (max_x-x)/window_size;
                }
                if(y < max_y-y){
                    dist_y = (y)/window_size;
                }
                else{
                    dist_y = (max_y-y)/window_size;
                }
                if(dist_x > 1.0)
                    dist_x = 1.0;
                if(dist_y > 1.0)
                    dist_y = 1.0;
                int weight = 0;
                if(dst_mask_.at<uchar>(x,y) == 255)
                    weight = 1;
                if(dist_x > dist_y){
                    dst_dt_.at<uchar>(x,y) = 255*weight;
                }
                else{
                    dst_dt_.at<uchar>(x,y) = 255*weight;
                }
            }
        }
        Mat uchar_dst;
        dst_.convertTo(uchar_dst,CV_8UC3);
        //Mat temp = dst_mask_.mul(dst_dt_);

        Mat out[] = {uchar_dst,dst_dt_};

        int channels_setting[] = {2,0, 1,1, 0,2, 3,3};
        __android_log_print(ANDROID_LOG_DEBUG,"C++ Composer","debug %d %d %d",dst_dt_.type(),uchar_dst.type(),dst_mask_.type());
        //mixChannels(out,2,&dst,1,channels_setting,4);
        __android_log_print(ANDROID_LOG_DEBUG,"C++ Composer","debug %d %d %d",dst_dt_.type(),uchar_dst.type(),dst_mask_.type());

//      dst = dst_;
        char img_file_name[50];
        sprintf(img_file_name,"/sdcard/stitch/pyramid%d.jpg",num_bands_);
        imwrite(img_file_name,dst_);

        dst_.assignTo(dst);
        dst_dt_.release();
        dst_mask_.release();
        dst_.release();
        uchar_dst.release();
    }
#endif
#ifdef ALPHA
    Mat dst_;
    Mat idst_, dst_mask_, dst_dt_;
    Rect dst_roi_;
    Mat dst_weight_map_, weight_map_;
    float sharpness_ = 0.02f;
    void prepare(Rect dst_roi,int not_used) {
        __android_log_print(ANDROID_LOG_DEBUG, "Composer", "Prepare Alpha");
        dst_.create(dst_roi.size(), CV_16SC3);
        dst_.setTo(Scalar::all(0));
        dst_mask_.create(dst_roi.size(), CV_8U);
        dst_mask_.setTo(Scalar::all(0));
        dst_roi_ = dst_roi;
        dst_dt_.create(dst_roi.size(), CV_8U);
        __android_log_print(ANDROID_LOG_DEBUG, "Composer", "Prepared");
        dst_weight_map_.create(dst_roi.size(), CV_32F);
        dst_weight_map_.setTo(0);
    }


    void feed(const Mat input_img,const Mat mask, Point tl) {
        Mat img;
        input_img.convertTo(img,CV_16SC3);
        CV_Assert(img.type() == CV_16SC3);
        CV_Assert(mask.type() == CV_8U);

        createWeightMap(mask, sharpness_, weight_map_);
        int dx = tl.x - dst_roi_.x;
        int dy = tl.y - dst_roi_.y;

        for (int y = 0; y < img.rows; ++y) {
            const Point3_<short> *src_row = img.ptr<Point3_<short> >(y);
            Point3_<short> *dst_row = dst_.ptr<Point3_<short> >(dy + y);
            const float *weight_row = weight_map_.ptr<float>(y);
            float *dst_weight_row = dst_weight_map_.ptr<float>(dy + y);

            for (int x = 0; x < img.cols; ++x) {
                dst_row[dx + x].x += static_cast<short>(src_row[x].x * weight_row[x]);
                dst_row[dx + x].y += static_cast<short>(src_row[x].y * weight_row[x]);
                dst_row[dx + x].z += static_cast<short>(src_row[x].z * weight_row[x]);
                dst_weight_row[dx + x] += weight_row[x];
            }
        }
    }


    void process(Mat &dst, Mat &dst_mask) {
        normalizeUsingWeightMap(dst_weight_map_, dst_);
        dst_mask_ = dst_weight_map_ > WEIGHT_EPS;
        __android_log_print(ANDROID_LOG_DEBUG,"Composer","Process Alpha");
        Mat temp1;
        dst_.convertTo(temp1,CV_8UC3);
        dst_ = temp1;
        int channels_setting[] = {2,0, 1,1, 0,2, 3,3};
        dst.create(dst_.size(),CV_8UC4);
        int max_x = dst_roi_.size().height;
        int max_y = dst_roi_.size().width;
        for(int x = 0 ; x < dst_roi_.size().height; x++){
            for(int y = 0 ; y < dst_roi_.size().width ; y++){
//                calc distance transform (replace 0.1)
                //large = avoid seam small = avoid ghost
                float window_size = 200.0;
                float dist_x = 0;
                float dist_y = 0;
                if(x < max_x-x){
                    dist_x = x/window_size;
                }
                else{
                    dist_x = (max_x-x)/window_size;
                }
                if(y < max_y-y){
                    dist_y = (y)/window_size;
                }
                else{
                    dist_y = (max_y-y)/window_size;
                }
                if(dist_x > 1.0)
                    dist_x = 1.0;
                if(dist_y > 1.0)
                    dist_y = 1.0;
                if(dist_x > dist_y){
                    dst_dt_.at<uchar>(x,y) = 255;
                }
                else{
                    dst_dt_.at<uchar>(x,y) = 255;
                }
            }
        }

        Mat temp = dst_mask_.mul(dst_dt_);

        Mat out[] = {dst_,temp};
        mixChannels(out,2,&dst,1,channels_setting,4);
//        dst = dst_;
        imwrite("/sdcard/stitch/alpha.jpg",dst_);
        dst_mask = temp;
        dst_dt_.release();
        dst_.release();
        dst_mask_.release();
    }


#endif

}