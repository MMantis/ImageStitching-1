#include "matcher.h"

namespace matcher {
    template<typename T> int icvCompressPoints( T* ptr, const uchar* mask, int mstep, int count )
    {
        int i, j;
        for( i = j = 0; i < count; i++ )
            if( mask[i*mstep] )
            {
                if( i > j )
                    ptr[j] = ptr[i];
                j++;
            }
        return j;
    }
    class CvModelEstimator2 {
                    public:
                    CvModelEstimator2(int _modelPoints, CvSize _modelSize, int _maxBasicSolutions);
                    virtual ~CvModelEstimator2();

                    virtual int runKernel( const CvMat* m1, const CvMat* m2, CvMat* model )=0;
                    virtual bool runLMeDS( const CvMat* m1, const CvMat* m2, CvMat* model,
                    CvMat* mask, double confidence=0.99, int maxIters=2000 );
                    virtual bool runRANSAC( const CvMat* m1, const CvMat* m2, CvMat* model,
                    CvMat* mask, double threshold,
                    double confidence=0.99, int maxIters=2000 );
                    virtual bool refine( const CvMat*, const CvMat*, CvMat*, int ) { return true; }
                    virtual void setSeed( int64 seed );

                    protected:
                    virtual void computeReprojError( const CvMat* m1, const CvMat* m2,
                    const CvMat* model, CvMat* error ) = 0;
                    virtual int findInliers( const CvMat* m1, const CvMat* m2,
                    const CvMat* model, CvMat* error,
                    CvMat* mask, double threshold );
                    virtual bool getSubset( const CvMat* m1, const CvMat* m2,
                    CvMat* ms1, CvMat* ms2, int maxAttempts=1000 );
                    virtual bool checkSubset( const CvMat* ms1, int count );

                    CvRNG rng;
                    int modelPoints;
                    CvSize modelSize;
                    int maxBasicSolutions;
                    bool checkPartialSubsets;
            };

    CvModelEstimator2::CvModelEstimator2(int _modelPoints, CvSize _modelSize, int _maxBasicSolutions)
    {
        modelPoints = _modelPoints;
        modelSize = _modelSize;
        maxBasicSolutions = _maxBasicSolutions;
        checkPartialSubsets = true;
        rng = cvRNG(-1);
    }

    CvModelEstimator2::~CvModelEstimator2()
    {
    }

    void CvModelEstimator2::setSeed( int64 seed )
    {
        rng = cvRNG(seed);
    }


    int CvModelEstimator2::findInliers( const CvMat* m1, const CvMat* m2,
                                        const CvMat* model, CvMat* _err,
                                        CvMat* _mask, double threshold )
    {
        int i, count = _err->rows*_err->cols, goodCount = 0;
        const float* err = _err->data.fl;
        uchar* mask = _mask->data.ptr;

        computeReprojError( m1, m2, model, _err );
        threshold *= threshold;
        for( i = 0; i < count; i++ )
            goodCount += mask[i] = err[i] <= threshold;
        return goodCount;
    }
    int cvRANSACUpdateNumIters( double p, double ep,
                            int model_points, int max_iters )
    {
        if( model_points <= 0 )
            CV_Error( CV_StsOutOfRange, "the number of model points should be positive" );

        p = MAX(p, 0.);
        p = MIN(p, 1.);
        ep = MAX(ep, 0.);
        ep = MIN(ep, 1.);

        // avoid inf's & nan's
        double num = MAX(1. - p, DBL_MIN);
        double denom = 1. - pow(1. - ep,model_points);
        if( denom < DBL_MIN )
            return 0;

        num = log(num);
        denom = log(denom);

        return denom >= 0 || -num >= max_iters*(-denom) ?
               max_iters : cvRound(num/denom);
    }

    bool CvModelEstimator2::runRANSAC( const CvMat* m1, const CvMat* m2, CvMat* model,
                                       CvMat* mask0, double reprojThreshold,
                                       double confidence, int maxIters )
    {
        bool result = false;
        cv::Ptr<CvMat> mask = cvCloneMat(mask0);
        cv::Ptr<CvMat> models, err, tmask;
        cv::Ptr<CvMat> ms1, ms2;

        int iter, niters = maxIters;
        int count = m1->rows*m1->cols, maxGoodCount = 0;
        CV_Assert( CV_ARE_SIZES_EQ(m1, m2) && CV_ARE_SIZES_EQ(m1, mask) );

        if( count < modelPoints )
            return false;

        models = cvCreateMat( modelSize.height*maxBasicSolutions, modelSize.width, CV_64FC1 );
        err = cvCreateMat( 1, count, CV_32FC1 );
        tmask = cvCreateMat( 1, count, CV_8UC1 );

        if( count > modelPoints )
        {
            ms1 = cvCreateMat( 1, modelPoints, m1->type );
            ms2 = cvCreateMat( 1, modelPoints, m2->type );
        }
        else
        {
            niters = 1;
            ms1 = cvCloneMat(m1);
            ms2 = cvCloneMat(m2);
        }

        for( iter = 0; iter < niters; iter++ )
        {
            int i, goodCount, nmodels;
            if( count > modelPoints )
            {
                bool found = getSubset( m1, m2, ms1, ms2, 300 );
                if( !found )
                {
                    if( iter == 0 )
                        return false;
                    break;
                }
            }

            nmodels = runKernel( ms1, ms2, models );
            if( nmodels <= 0 )
                continue;
            for( i = 0; i < nmodels; i++ )
            {
                CvMat model_i;
                cvGetRows( models, &model_i, i*modelSize.height, (i+1)*modelSize.height );
                goodCount = findInliers( m1, m2, &model_i, err, tmask, reprojThreshold );

                if( goodCount > MAX(maxGoodCount, modelPoints-1) )
                {
                    std::swap(tmask, mask);
                    cvCopy( &model_i, model );
                    maxGoodCount = goodCount;
                    niters = cvRANSACUpdateNumIters( confidence,
                                                     (double)(count - goodCount)/count, modelPoints, niters );
                }
            }
        }

        if( maxGoodCount > 0 )
        {
            if( mask != mask0 )
                cvCopy( mask, mask0 );
            result = true;
        }

        return result;
    }

    //Not yet implements
    bool CvModelEstimator2::runLMeDS( const CvMat* m1, const CvMat* m2, CvMat* model,
                                      CvMat* mask, double confidence, int maxIters )
    {
        return false;
    }


    bool CvModelEstimator2::getSubset( const CvMat* m1, const CvMat* m2,
                                       CvMat* ms1, CvMat* ms2, int maxAttempts )
    {
        cv::AutoBuffer<int> _idx(modelPoints);
        int* idx = _idx;
        int i = 0, j, k, idx_i, iters = 0;
        int type = CV_MAT_TYPE(m1->type), elemSize = CV_ELEM_SIZE(type);
        const int *m1ptr = m1->data.i, *m2ptr = m2->data.i;
        int *ms1ptr = ms1->data.i, *ms2ptr = ms2->data.i;
        int count = m1->cols*m1->rows;

        assert( CV_IS_MAT_CONT(m1->type & m2->type) && (elemSize % sizeof(int) == 0) );
        elemSize /= sizeof(int);

        for(; iters < maxAttempts; iters++)
        {
            for( i = 0; i < modelPoints && iters < maxAttempts; )
            {
                idx[i] = idx_i = cvRandInt(&rng) % count;
                for( j = 0; j < i; j++ )
                    if( idx_i == idx[j] )
                        break;
                if( j < i )
                    continue;
                for( k = 0; k < elemSize; k++ )
                {
                    ms1ptr[i*elemSize + k] = m1ptr[idx_i*elemSize + k];
                    ms2ptr[i*elemSize + k] = m2ptr[idx_i*elemSize + k];
                }
                if( checkPartialSubsets && (!checkSubset( ms1, i+1 ) || !checkSubset( ms2, i+1 )))
                {
                    iters++;
                    continue;
                }
                i++;
            }
            if( !checkPartialSubsets && i == modelPoints &&
                (!checkSubset( ms1, i ) || !checkSubset( ms2, i )))
                continue;
            break;
        }

        return i == modelPoints && iters < maxAttempts;
    }


    bool CvModelEstimator2::checkSubset( const CvMat* m, int count )
    {
        if( count <= 2 )
            return true;

        int j, k, i, i0, i1;
        CvPoint2D64f* ptr = (CvPoint2D64f*)m->data.ptr;

        assert( CV_MAT_TYPE(m->type) == CV_64FC2 );

        if( checkPartialSubsets )
            i0 = i1 = count - 1;
        else
            i0 = 0, i1 = count - 1;

        for( i = i0; i <= i1; i++ )
        {
            // check that the i-th selected point does not belong
            // to a line connecting some previously selected points
            for( j = 0; j < i; j++ )
            {
                double dx1 = ptr[j].x - ptr[i].x;
                double dy1 = ptr[j].y - ptr[i].y;
                for( k = 0; k < j; k++ )
                {
                    double dx2 = ptr[k].x - ptr[i].x;
                    double dy2 = ptr[k].y - ptr[i].y;
                    if( fabs(dx2*dy1 - dy2*dx1) <= FLT_EPSILON*(fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
                        break;
                }
                if( k < j )
                    break;
            }
            if( j < i )
                break;
        }

        return i > i1;
    }
    class CvHomographyEstimator : public CvModelEstimator2
    {
    public:
        CvHomographyEstimator( int modelPoints );

        virtual int runKernel( const CvMat* m1, const CvMat* m2, CvMat* model );
        virtual bool refine( const CvMat* m1, const CvMat* m2,
                             CvMat* model, int maxIters );
    protected:
        virtual void computeReprojError( const CvMat* m1, const CvMat* m2,
                                         const CvMat* model, CvMat* error );
    };


    CvHomographyEstimator::CvHomographyEstimator(int _modelPoints)
            : CvModelEstimator2(_modelPoints, cvSize(3,3), 1)
    {
        assert( _modelPoints == 4 || _modelPoints == 5 );
        checkPartialSubsets = false;
    }

    int CvHomographyEstimator::runKernel( const CvMat* m1, const CvMat* m2, CvMat* H )
    {
        int i, count = m1->rows*m1->cols;
        const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
        const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;

        double LtL[9][9], W[9][1], V[9][9];
        CvMat _LtL = cvMat( 9, 9, CV_64F, LtL );
        CvMat matW = cvMat( 9, 1, CV_64F, W );
        CvMat matV = cvMat( 9, 9, CV_64F, V );
        CvMat _H0 = cvMat( 3, 3, CV_64F, V[8] );
        CvMat _Htemp = cvMat( 3, 3, CV_64F, V[7] );
        CvPoint2D64f cM={0,0}, cm={0,0}, sM={0,0}, sm={0,0};

        for( i = 0; i < count; i++ )
        {
            cm.x += m[i].x; cm.y += m[i].y;
            cM.x += M[i].x; cM.y += M[i].y;
        }

        cm.x /= count; cm.y /= count;
        cM.x /= count; cM.y /= count;

        for( i = 0; i < count; i++ )
        {
            sm.x += fabs(m[i].x - cm.x);
            sm.y += fabs(m[i].y - cm.y);
            sM.x += fabs(M[i].x - cM.x);
            sM.y += fabs(M[i].y - cM.y);
        }

        if( fabs(sm.x) < DBL_EPSILON || fabs(sm.y) < DBL_EPSILON ||
            fabs(sM.x) < DBL_EPSILON || fabs(sM.y) < DBL_EPSILON )
            return 0;
        sm.x = count/sm.x; sm.y = count/sm.y;
        sM.x = count/sM.x; sM.y = count/sM.y;

        double invHnorm[9] = { 1./sm.x, 0, cm.x, 0, 1./sm.y, cm.y, 0, 0, 1 };
        double Hnorm2[9] = { sM.x, 0, -cM.x*sM.x, 0, sM.y, -cM.y*sM.y, 0, 0, 1 };
        CvMat _invHnorm = cvMat( 3, 3, CV_64FC1, invHnorm );
        CvMat _Hnorm2 = cvMat( 3, 3, CV_64FC1, Hnorm2 );

        cvZero( &_LtL );
        for( i = 0; i < count; i++ )
        {
            double x = (m[i].x - cm.x)*sm.x, y = (m[i].y - cm.y)*sm.y;
            double X = (M[i].x - cM.x)*sM.x, Y = (M[i].y - cM.y)*sM.y;
            double Lx[] = { X, Y, 1, 0, 0, 0, -x*X, -x*Y, -x };
            double Ly[] = { 0, 0, 0, X, Y, 1, -y*X, -y*Y, -y };
            int j, k;
            for( j = 0; j < 9; j++ )
                for( k = j; k < 9; k++ )
                    LtL[j][k] += Lx[j]*Lx[k] + Ly[j]*Ly[k];
        }
        cvCompleteSymm( &_LtL );

        //cvSVD( &_LtL, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T );
        cvEigenVV( &_LtL, &matV, &matW );
        cvMatMul( &_invHnorm, &_H0, &_Htemp );
        cvMatMul( &_Htemp, &_Hnorm2, &_H0 );
        cvConvertScale( &_H0, H, 1./_H0.data.db[8] );

        return 1;
    }


    void CvHomographyEstimator::computeReprojError( const CvMat* m1, const CvMat* m2,
                                                    const CvMat* model, CvMat* _err )
    {
        int i, count = m1->rows*m1->cols;
        const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
        const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
        const double* H = model->data.db;
        float* err = _err->data.fl;

        for( i = 0; i < count; i++ )
        {
            double ww = 1./(H[6]*M[i].x + H[7]*M[i].y + 1.);
            double dx = (H[0]*M[i].x + H[1]*M[i].y + H[2])*ww - m[i].x;
            double dy = (H[3]*M[i].x + H[4]*M[i].y + H[5])*ww - m[i].y;
            err[i] = (float)(dx*dx + dy*dy);
        }
    }

    bool CvHomographyEstimator::refine( const CvMat* m1, const CvMat* m2, CvMat* model, int maxIters )
    {
        CvLevMarq solver(8, 0, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, maxIters, DBL_EPSILON));
        int i, j, k, count = m1->rows*m1->cols;
        const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
        const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
        CvMat modelPart = cvMat( solver.param->rows, solver.param->cols, model->type, model->data.ptr );
        cvCopy( &modelPart, solver.param );

        for(;;)
        {
            const CvMat* _param = 0;
            CvMat *_JtJ = 0, *_JtErr = 0;
            double* _errNorm = 0;

            if( !solver.updateAlt( _param, _JtJ, _JtErr, _errNorm ))
                break;

            for( i = 0; i < count; i++ )
            {
                const double* h = _param->data.db;
                double Mx = M[i].x, My = M[i].y;
                double ww = h[6]*Mx + h[7]*My + 1.;
                ww = fabs(ww) > DBL_EPSILON ? 1./ww : 0;
                double _xi = (h[0]*Mx + h[1]*My + h[2])*ww;
                double _yi = (h[3]*Mx + h[4]*My + h[5])*ww;
                double err[] = { _xi - m[i].x, _yi - m[i].y };
                if( _JtJ || _JtErr )
                {
                    double J[][8] =
                            {
                                    { Mx*ww, My*ww, ww, 0, 0, 0, -Mx*ww*_xi, -My*ww*_xi },
                                    { 0, 0, 0, Mx*ww, My*ww, ww, -Mx*ww*_yi, -My*ww*_yi }
                            };

                    for( j = 0; j < 8; j++ )
                    {
                        for( k = j; k < 8; k++ )
                            _JtJ->data.db[j*8+k] += J[0][j]*J[0][k] + J[1][j]*J[1][k];
                        _JtErr->data.db[j] += J[0][j]*err[0] + J[1][j]*err[1];
                    }
                }
                if( _errNorm )
                    *_errNorm += err[0]*err[0] + err[1]*err[1];
            }
        }

        cvCopy( solver.param, &modelPart );
        return true;
    }




    int num_matches_thresh1_;
    int num_matches_thresh2_;
    Ptr <FeaturesMatcher> impl_;
    float match_conf_;
    typedef set <pair<int, int>> MatchesSet;
    void cpuMatch(const ImageFeatures &features1, const ImageFeatures &features2,
                  MatchesInfo &matches_info, float match_conf_) {
        CV_Assert(features1.descriptors.type() == features2.descriptors.type());
        CV_Assert(
                features2.descriptors.depth() == CV_8U || features2.descriptors.depth() == CV_32F);

        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.1.1");
        matches_info.matches.clear();

        Ptr <flann::IndexParams> indexParams = new flann::KDTreeIndexParams();
        Ptr <flann::SearchParams> searchParams = new flann::SearchParams();

        if (features2.descriptors.depth() == CV_8U) {
            indexParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
            searchParams->setAlgorithm(cvflann::FLANN_INDEX_LSH);
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.1.2");
        FlannBasedMatcher matcher(indexParams, searchParams);
        vector <vector<DMatch>> pair_matches;
        MatchesSet matches;

        // Find 1->2 matches
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.1.3");
        matcher.knnMatch(features1.descriptors, features2.descriptors, pair_matches, 2);
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf_) * m1.distance) {
                matches_info.matches.push_back(m0);
                matches.insert(make_pair(m0.queryIdx, m0.trainIdx));
            }
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.1.4");
        // Find 2->1 matches
        pair_matches.clear();
        matcher.knnMatch(features2.descriptors, features1.descriptors, pair_matches, 2);
        for (size_t i = 0; i < pair_matches.size(); ++i) {
            if (pair_matches[i].size() < 2)
                continue;
            const DMatch &m0 = pair_matches[i][0];
            const DMatch &m1 = pair_matches[i][1];
            if (m0.distance < (1.f - match_conf_) * m1.distance) if (
                    matches.find(make_pair(m0.trainIdx, m0.queryIdx)) == matches.end())
                matches_info.matches.push_back(DMatch(m0.trainIdx, m0.queryIdx, m0.distance));
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.1.5");
    }

    void create(float match_conf, int num_matches_thresh1, int num_matches_thresh2) {
        match_conf_ = match_conf;
        num_matches_thresh1_ = num_matches_thresh1;
        num_matches_thresh2_ = num_matches_thresh2;
    }


    void match(const vector<ImageFeatures> &features, vector<MatchesInfo> &pairwise_matches,int newIndex)
    {
        const int num_images = static_cast<int>(features.size());
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "1");
        Mat_<uchar> mask_;
        mask_ = Mat::zeros(num_images, num_images, CV_8U);
        for(int i = 0; i < num_images ;i++){
            mask_.at<uchar>(i,newIndex) = 1;
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "2");
        vector<pair<int,int> > near_pairs;
        for (int i = 0; i < num_images - 1; ++i)
            for (int j = i + 1; j < num_images; ++j)
                if (features[i].keypoints.size() > 0 && features[j].keypoints.size() > 0 && mask_(i, j))
                    near_pairs.push_back(make_pair(i, j));

        pairwise_matches.resize(num_images * num_images);
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3");
        for (int i = 0; i < near_pairs.size(); ++i)
        {
            int from = near_pairs[i].first;
            int to = near_pairs[i].second;
            int pair_idx = from*num_images + to;

            match(features[from], features[to], pairwise_matches[pair_idx]);
            pairwise_matches[pair_idx].src_img_idx = from;
            pairwise_matches[pair_idx].dst_img_idx = to;

            size_t dual_pair_idx = to*num_images + from;

            pairwise_matches[dual_pair_idx] = pairwise_matches[pair_idx];
            pairwise_matches[dual_pair_idx].src_img_idx = to;
            pairwise_matches[dual_pair_idx].dst_img_idx = from;

            if (!pairwise_matches[pair_idx].H.empty())
                pairwise_matches[dual_pair_idx].H = pairwise_matches[pair_idx].H.inv();

            for (size_t j = 0; j < pairwise_matches[dual_pair_idx].matches.size(); ++j)
                std::swap(pairwise_matches[dual_pair_idx].matches[j].queryIdx,
                          pairwise_matches[dual_pair_idx].matches[j].trainIdx);
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "4");
        for(int i = 0 ; i < pairwise_matches.size(); i++){
            if(pairwise_matches[i].matches.size()!= 0 && pairwise_matches[i].src_img_idx < pairwise_matches[i].dst_img_idx)
            __android_log_print(ANDROID_LOG_DEBUG,"RANSAC","%d:%d (%d,%d)",pairwise_matches[i].src_img_idx,pairwise_matches[i].dst_img_idx,pairwise_matches[i].matches.size(),pairwise_matches[i].num_inliers);
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "5");

//        MatchPairsBody body(*this, features, pairwise_matches, near_pairs);
//        body(Range(0, static_cast<int>(near_pairs.size())));

    }

    void match(const vector<ImageFeatures> &features, vector<MatchesInfo> &pairwise_matches)
    {
        const int num_images = static_cast<int>(features.size());

        Mat_<uchar> mask_;
        mask_ = Mat::ones(num_images, num_images, CV_8U);

        vector<pair<int,int> > near_pairs;
        for (int i = 0; i < num_images - 1; ++i)
            for (int j = i + 1; j < num_images; ++j)
                if (features[i].keypoints.size() > 0 && features[j].keypoints.size() > 0 && mask_(i, j))
                    near_pairs.push_back(make_pair(i, j));

        pairwise_matches.resize(num_images * num_images);

        for (int i = 0; i < near_pairs.size(); ++i)
        {
            int from = near_pairs[i].first;
            int to = near_pairs[i].second;
            int pair_idx = from*num_images + to;

            match(features[from], features[to], pairwise_matches[pair_idx]);
            pairwise_matches[pair_idx].src_img_idx = from;
            pairwise_matches[pair_idx].dst_img_idx = to;

            size_t dual_pair_idx = to*num_images + from;

            pairwise_matches[dual_pair_idx] = pairwise_matches[pair_idx];
            pairwise_matches[dual_pair_idx].src_img_idx = to;
            pairwise_matches[dual_pair_idx].dst_img_idx = from;

            if (!pairwise_matches[pair_idx].H.empty())
                pairwise_matches[dual_pair_idx].H = pairwise_matches[pair_idx].H.inv();

            for (size_t j = 0; j < pairwise_matches[dual_pair_idx].matches.size(); ++j)
                std::swap(pairwise_matches[dual_pair_idx].matches[j].queryIdx,
                          pairwise_matches[dual_pair_idx].matches[j].trainIdx);
        }

        for(int i = 0 ; i < pairwise_matches.size(); i++){
            if(pairwise_matches[i].matches.size()!= 0 && pairwise_matches[i].src_img_idx < pairwise_matches[i].dst_img_idx)
                __android_log_print(ANDROID_LOG_DEBUG,"RANSAC","%d:%d (%d,%d)",pairwise_matches[i].src_img_idx,pairwise_matches[i].dst_img_idx,pairwise_matches[i].matches.size(),pairwise_matches[i].num_inliers);
        }


//        MatchPairsBody body(*this, features, pairwise_matches, near_pairs);
//        body(Range(0, static_cast<int>(near_pairs.size())));

    }




    void match(const ImageFeatures &features1, const ImageFeatures &features2,
               MatchesInfo &matches_info) {

        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.1");
        cpuMatch(features1, features2, matches_info, match_conf_);
        // Check if it makes sense to find homography
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.2");
        if (matches_info.matches.size() < static_cast<size_t>(num_matches_thresh1_))
            return;

        // Construct point-point correspondences for homography estimation
        Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
        Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.3");
        for (size_t i = 0; i < matches_info.matches.size(); ++i) {
            const DMatch &m = matches_info.matches[i];

            Point2f p = features1.keypoints[m.queryIdx].pt;
            p.x -= features1.img_size.width * 0.5f;
            p.y -= features1.img_size.height * 0.5f;
            src_points.at<Point2f>(0, static_cast<int>(i)) = p;

            p = features2.keypoints[m.trainIdx].pt;
            p.x -= features2.img_size.width * 0.5f;
            p.y -= features2.img_size.height * 0.5f;
            dst_points.at<Point2f>(0, static_cast<int>(i)) = p;
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4");
        // Find pair-wise motion
        matches_info.H = matcher::findHomography(src_points, dst_points, CV_RANSAC, 5.0, matches_info.inliers_mask);
        if (std::abs(determinant(matches_info.H)) < numeric_limits<double>::epsilon())
            return;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.5");
        // Find number of inliers
        matches_info.num_inliers = 0;
        for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
            if (matches_info.inliers_mask[i])
                matches_info.num_inliers++;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.6");
        // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
        // using Invariant Features"
        matches_info.confidence =
                matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());

        // Set zero confidence to remove matches between too close images, as they don't provide
        // additional information anyway. The threshold was set experimentally.
        matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;

        // Check if we should try to refine motion
        if (matches_info.num_inliers < num_matches_thresh2_)
            return;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.7");
        // Construct point-point correspondences for inliers only
        src_points.create(1, matches_info.num_inliers, CV_32FC2);
        dst_points.create(1, matches_info.num_inliers, CV_32FC2);
        int inlier_idx = 0;
        for (size_t i = 0; i < matches_info.matches.size(); ++i) {
            if (!matches_info.inliers_mask[i])
                continue;

            const DMatch &m = matches_info.matches[i];
            Point2f p = features1.keypoints[m.queryIdx].pt;
            p.x -= features1.img_size.width * 0.5f;
            p.y -= features1.img_size.height * 0.5f;
            src_points.at<Point2f>(0, inlier_idx) = p;

            p = features2.keypoints[m.trainIdx].pt;
            p.x -= features2.img_size.width * 0.5f;
            p.y -= features2.img_size.height * 0.5f;
            dst_points.at<Point2f>(0, inlier_idx) = p;

            inlier_idx++;
        }
        __android_log_print(ANDROID_LOG_VERBOSE,"C++ Matcher","RANSAC inliner A:I [%d : %d]",matches_info.matches.size(),matches_info.num_inliers);
        // Rerun motion estimation on inliers only
        matches_info.H = matcher::findHomography(src_points, dst_points, 0);
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.8");
    }




    void forceMatch(const ImageFeatures &features1, const ImageFeatures &features2, MatchesInfo &matches_info) {
        for(int i = 0 ; i < features1.keypoints.size();i++){
            matches_info.matches.push_back(DMatch(i,i,1));
        }

        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.1");
        // Check if it makes sense to find homography
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.2");
        if (matches_info.matches.size() < static_cast<size_t>(num_matches_thresh1_))
            return;

        // Construct point-point correspondences for homography estimation
        Mat src_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
        Mat dst_points(1, static_cast<int>(matches_info.matches.size()), CV_32FC2);
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.3");
        for (size_t i = 0; i < matches_info.matches.size(); ++i) {
            const DMatch &m = matches_info.matches[i];
            Point2f p = features1.keypoints[m.queryIdx].pt;
            p.x -= features1.img_size.width * 0.5f;
            p.y -= features1.img_size.height * 0.5f;
            src_points.at<Point2f>(0, static_cast<int>(i)) = p;

            p = features2.keypoints[m.trainIdx].pt;
            p.x -= features2.img_size.width * 0.5f;
            p.y -= features2.img_size.height * 0.5f;
            dst_points.at<Point2f>(0, static_cast<int>(i)) = p;
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4");
        // Find pair-wise motion
        matches_info.H = matcher::findHomography(src_points, dst_points, CV_RANSAC, 5.0, matches_info.inliers_mask);
        if (std::abs(determinant(matches_info.H)) < numeric_limits<double>::epsilon())
            return;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.5");
        // Find number of inliers
        matches_info.num_inliers = 0;
        for (size_t i = 0; i < matches_info.inliers_mask.size(); ++i)
            if (matches_info.inliers_mask[i])
                matches_info.num_inliers++;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.6");
        // These coeffs are from paper M. Brown and D. Lowe. "Automatic Panoramic Image Stitching
        // using Invariant Features"
        matches_info.confidence =
                matches_info.num_inliers / (8 + 0.3 * matches_info.matches.size());

        // Set zero confidence to remove matches between too close images, as they don't provide
        // additional information anyway. The threshold was set experimentally.
        matches_info.confidence = matches_info.confidence > 3. ? 0. : matches_info.confidence;

        // Check if we should try to refine motion
        if (matches_info.num_inliers < num_matches_thresh2_)
            return;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.7");
        // Construct point-point correspondences for inliers only
        src_points.create(1, matches_info.num_inliers, CV_32FC2);
        dst_points.create(1, matches_info.num_inliers, CV_32FC2);
        int inlier_idx = 0;
        for (size_t i = 0; i < matches_info.matches.size(); ++i) {
            if (!matches_info.inliers_mask[i])
                continue;

            const DMatch &m = matches_info.matches[i];
            Point2f p = features1.keypoints[m.queryIdx].pt;
            p.x -= features1.img_size.width * 0.5f;
            p.y -= features1.img_size.height * 0.5f;
            src_points.at<Point2f>(0, inlier_idx) = p;

            p = features2.keypoints[m.trainIdx].pt;
            p.x -= features2.img_size.width * 0.5f;
            p.y -= features2.img_size.height * 0.5f;
            dst_points.at<Point2f>(0, inlier_idx) = p;

            inlier_idx++;
        }
        __android_log_print(ANDROID_LOG_VERBOSE,"C++ Matcher","RANSAC inliner A:I [%d : %d]",matches_info.matches.size(),matches_info.num_inliers);
        // Rerun motion estimation on inliers only
        matches_info.H = matcher::findHomography(src_points, dst_points, 0);
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.8");
    }

    void collectGarbage() {
    }



    cv::Mat findHomography( InputArray _points1, InputArray _points2,
                            int method, double ransacReprojThreshold, OutputArray _mask )
    {
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.0");
        Mat points1 = _points1.getMat(), points2 = _points2.getMat();
        int npoints = points1.checkVector(2);
        CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
                   points1.type() == points2.type());
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.01");
        Mat H(3, 3, CV_64F);
        CvMat _pt1 = points1, _pt2 = points2;
        CvMat matH = H, c_mask, *p_mask = 0;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.02");
        if( _mask.needed() )
        {
            _mask.create(npoints, 1, CV_8U, -1, true);
            p_mask = &(c_mask = _mask.getMat());
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.03");
        bool ok = cvFindHomography2( &_pt1, &_pt2, &matH, method, ransacReprojThreshold, p_mask ) > 0;
        if( !ok )
            H = Scalar(0);
        return H;
    }

    cv::Mat findHomography( InputArray _points1, InputArray _points2,int method)
    {
        vector<uchar> out;
        return cv::findHomography(_points1, _points2, method, 0, out);
    }

    int cvFindHomography2( const CvMat* objectPoints, const CvMat* imagePoints,
                      CvMat* __H, int method, double ransacReprojThreshold,
                      CvMat* mask )
    {
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.1");
        const double confidence = 0.995;
        const int maxIters = 2000;
        const double defaultRANSACReprojThreshold = 3;
        bool result = false;
        Ptr<CvMat> m, M, tempMask;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.2");
        double H[9];
        CvMat matH = cvMat( 3, 3, CV_64FC1, H );
        int count;

        CV_Assert( CV_IS_MAT(imagePoints) && CV_IS_MAT(objectPoints) );

        count = MAX(imagePoints->cols, imagePoints->rows);
        CV_Assert( count >= 4 );
        if( ransacReprojThreshold <= 0 )
            ransacReprojThreshold = defaultRANSACReprojThreshold;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.3");
        m = cvCreateMat( 1, count, CV_64FC2 );
        cvConvertPointsHomogeneous( imagePoints, m );

        M = cvCreateMat( 1, count, CV_64FC2 );
        cvConvertPointsHomogeneous( objectPoints, M );
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.4");
        if( mask )
        {
            CV_Assert( CV_IS_MASK_ARR(mask) && CV_IS_MAT_CONT(mask->type) &&
                       (mask->rows == 1 || mask->cols == 1) &&
                       mask->rows*mask->cols == count );
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.5");
        if( mask || count > 4 )
            tempMask = cvCreateMat( 1, count, CV_8U );
        if( !tempMask.empty() )
            cvSet( tempMask, cvScalarAll(1.) );
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.6");
        CvHomographyEstimator estimator(4);
        if( count == 4 )
            method = 0;
        if( method == CV_LMEDS )
            result = estimator.runLMeDS( M, m, &matH, tempMask, confidence, maxIters );
        else if( method == CV_RANSAC )
            result = estimator.runRANSAC( M, m, &matH, tempMask, ransacReprojThreshold, confidence, maxIters);
        else
            result = estimator.runKernel( M, m, &matH ) > 0;
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.7");
        if( result && count > 4 )
        {
            icvCompressPoints( (CvPoint2D64f*)M->data.ptr, tempMask->data.ptr, 1, count );
            count = icvCompressPoints( (CvPoint2D64f*)m->data.ptr, tempMask->data.ptr, 1, count );
            M->cols = m->cols = count;
            if( method == CV_RANSAC )
                estimator.runKernel( M, m, &matH );
            estimator.refine( M, m, &matH, 10 );
        }
        __android_log_print(ANDROID_LOG_DEBUG, "C++ Stitching", "3.4.8");
        if( result )
            cvConvert( &matH, __H );

        if( mask && tempMask )
        {
            if( CV_ARE_SIZES_EQ(mask, tempMask) )
                cvCopy( tempMask, mask );
            else
                cvTranspose( tempMask, mask );
        }

        return (int)result;
    }




}
