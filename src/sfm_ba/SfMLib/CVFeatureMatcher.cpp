#include "CVFeatureMatcher.h"

/* ------------------------------------------------------------------------- */
/* \fn  CVFeatureMatcher
*
* \brief    constructor for featurematcher
*/
/* ------------------------------------------------------------------------- */
CVFeatureMatcher::
CVFeatureMatcher()
{
    _detector_method = "SIFT";
    _extractor_method = "SIFT";
    _detector = cv::FeatureDetector::create("SIFT");
    _extractor = cv::DescriptorExtractor::create("SIFT");
    if(_detector.empty())
    {
        SfM_ERROR("_detector with method -"+_detector_method+"- is empty");
        CLASS_CREATE_MSG("FeatcherMatcher", false);
        return;
    }
    if(_extractor.empty())
    {
        SfM_ERROR("_extractor with method -"+_extractor_method+"- is empty");
        CLASS_CREATE_MSG("FeatcherMatcher", false);
        return;
    }
    CLASS_CREATE_MSG("FeatcherMatcher", true);

    _init_allready = false;
    _dofeaturematching_allready = false;
}

/* ------------------------------------------------------------------------- */
/* \fn  CVFeatureMatcher
*
* \brief    constructor for featurematcher
*
* \param[in]    FeatureDetecteMethod --> detector method for cv::FeatureDetector
*               DescriptorExtractorMethod --> extractor method for cv::DescriptorExtractor
*/
/* ------------------------------------------------------------------------- */
CVFeatureMatcher::
CVFeatureMatcher(const std::string FeatureDetecteMethod, const std::string DescriptorExtractorMethod)
{
    _detector_method = FeatureDetecteMethod;
    _extractor_method = DescriptorExtractorMethod;
    _detector = cv::FeatureDetector::create(FeatureDetecteMethod);
    _extractor = cv::DescriptorExtractor::create(DescriptorExtractorMethod);
    if(_detector.empty())
    {
        SfM_ERROR("_detector with method -"+_detector_method+"- is empty");
        CLASS_CREATE_MSG("FeatcherMatcher", false);
        return;
    }
    if(_extractor.empty())
    {
        SfM_ERROR("_extractor with method -"+_extractor_method+"- is empty");
        CLASS_CREATE_MSG("FeatcherMatcher", false);
        return;
    }
    CLASS_CREATE_MSG("self defined FeatcherMatcher", true);

    _init_allready = false;
    _dofeaturematching_allready = false;
}

/* ------------------------------------------------------------------------- */
/* \fn  bool Init
*
* \brief    init feature matcher parameter members.
*
* \param[in]    image_pre --> image at time 1
*               image_nxt --> image at time 2
*
* \return   if no parameter error return true , otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool CVFeatureMatcher::
Init(const cv::Mat image_pre, const cv::Mat image_nxt)
{
    if(image_pre.data && image_nxt.data)
    {
        _image_pre = image_pre.clone();
        _image_nxt = image_nxt.clone();
        _init_allready = true;
        SfM_SENDMSG("FeatureMatchter Init is complete");
    }else
    {
        SfM_ERROR("image_pre or image_nxt is empty");
        return false;
    }
    _dofeaturematching_allready = false;
    return true;
}


/* ------------------------------------------------------------------------- */
/* \fn  bool DoFeatureMatching
*
* \brief    do featurematching for 2 images
*           pipeline in my thesis
*
* \return   if seccessfult return true, otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool CVFeatureMatcher::
DoFeatureMatching()
{
    if(!_init_allready)
    {
        SfM_ERROR("FeatureMatching has been not inited");
        return false;
    }
    double time = cv::getTickCount();

    /* get keypnts */
    _keypnts_pre.clear();   _keypnts_nxt.clear();
    {
        _detector->detect(_image_pre, _keypnts_pre);
        _detector->detect(_image_nxt, _keypnts_nxt);

        _extractor->compute(_image_pre, _keypnts_pre, _descriptor_pre);
        _extractor->compute(_image_nxt, _keypnts_nxt, _descriptor_nxt);

        SfM_SENDMSG("_image_pre has " + std::to_string(_keypnts_pre.size()) + " key points");
        SfM_SENDMSG("_image_nxt has " + std::to_string(_keypnts_nxt.size()) + " key points");

        SfM_SENDMSG("feature-detection and compute-descriptor are complete");

        if(_descriptor_pre.empty() || _descriptor_nxt.empty())
        {
            SfM_ERROR("_descriptor_pre or_nxt is empty");
            return false;
        }
    }

    /* use Brute-Force Matcher */
     _matches.clear(); std::vector<double> dists;
    {
        if(_detector_method == "PyramidFAST" && _extractor_method == "ORB")
        {
            cv::BFMatcher matcher(cv::NORM_HAMMING,true); //allow cross-check
            if (_matches.size() == 0)
            {
                std::vector<std::vector<cv::DMatch> > nn_matches;
                matcher.knnMatch(_descriptor_pre, _descriptor_nxt, nn_matches, 1);
                _matches.clear();
                for(int i = 0; i < nn_matches.size(); i++)
                {
                    if(nn_matches[i].size() > 0)
                    {
                        _matches.push_back(nn_matches[i][0]);
                        double dist = _matches.back().distance;
                        if(fabs(dist) > 10000)  dist = 1.0;
                        _matches.back().distance = dist;
                        dists.push_back(dist);
                    }
                }
            }
        }
        else
        {
            cv::BFMatcher matcher(cv::NORM_L2,true);
#if DEBUG
            SfM_SENDMSG("use matcher(cv::NORM_L2,true)");
#endif
            matcher.match(_descriptor_pre, _descriptor_nxt, _matches);
            for(int i = 0; i < _matches.size(); i++)
            {
                dists.push_back(_matches[i].distance);
            }
        }
    }

    /* matches filter , find good matches */
    cv::vector<cv::DMatch> matches_good;
    {
        double max_dist = 0.0; double min_dist = 0.0;
        cv::minMaxIdx(dists, &min_dist, &max_dist);

        float block_num = 30.0;
        std::vector<int> histo_gram(block_num,0);
        const float block = (max_dist-min_dist)/block_num;

        for(int i = 0; i < _matches.size(); i++)
        {
           for(int j = 0; j < block_num; j++)
           {
               if(_matches[i].distance < min_dist+block*(j+1))
               {
                    histo_gram[j]++;
                    break;
               }
           }
        }
        double max_block_value = 0; int max_block_num = 0;
        for(int i = 0; i < block_num; i++)
        {
            if(max_block_value < histo_gram[i])
            {
                max_block_value = histo_gram[i];
                max_block_num = i;
            }
#if DEBUG
            std::cout << histo_gram[i] << "," ;
#endif
        }
#if DEBUG
        std::cout << std::endl;
#endif
        const int block_move = int(max_block_num/2);
#if DEBUG
        SfM_SENDMSG("max and min distance in matches are "+std::to_string(max_dist)+", "+std::to_string(min_dist));
        SfM_SENDMSG("for good filter value finding, use histogram");
        SfM_SENDMSG("histogram block num : "+std::to_string(block_num));
        SfM_SENDMSG("histogram block size : "+std::to_string(block));
        SfM_SENDMSG("histogram max element : "+std::to_string(max_block_value)+" at position "+std::to_string(max_block_num));
        SfM_SENDMSG("histogram method value choice block move value : "+std::to_string(block_move))
#endif
#if DEBUG
        bool cutoff_choose_histo = false;
#endif
        double cutoff = 0;
        {
            if(max_block_num != block_num-1 && max_block_num+block_move < block_num)
            {
                cutoff = min_dist+(max_block_num+block_move)*block;
#if DEBUG
                cutoff_choose_histo = true;
#endif
            }
            else
                cutoff = 0.3*(min_dist+max_dist);
        }
#if DEBUG
        SfM_SENDMSG("max, min distance is : "+std::to_string(max_dist)+", "+std::to_string(min_dist));
        SfM_SENDMSG("cutoff value : "+std::to_string(cutoff));
        if(cutoff_choose_histo)
        {   SfM_SENDMSG("cutoff value has choosen the histo method value "); }
        else
        {   SfM_SENDMSG("cutoff value has choosen the default value ");  }
#endif
        std::vector<int> existing_trainIdx(_keypnts_nxt.size(), 0);
        for(unsigned int i = 0; i < _matches.size(); i++ )
        {
            if( _matches[i].trainIdx <= 0 )
            {
                _matches[i].trainIdx = _matches[i].imgIdx;
            }

            int tidx = _matches[i].trainIdx;
            if( _matches[i].distance > 0.0 && _matches[i].distance < cutoff )
            {
                if( existing_trainIdx[tidx] == 0 &&
                    tidx >= 0 && tidx < (int)(_keypnts_nxt.size()) )
                {
                    matches_good.push_back(_matches[i]);
                    existing_trainIdx[tidx] = 1;
                }
            }
        }
    }
    _dofeaturematching_allready = true;
    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    SfM_SENDMSG("Done. (" + std::to_string(time) +"s)");
    SfM_SENDMSG("Matching is complete : find " + std::to_string(matches_good.size()) +
                   " good matches from " + std::to_string(_matches.size()) + " matches");

    StoreGoodMachtes(matches_good);
    std::vector<cv::KeyPoint> keypnts_good_pre, keypnts_good_nxt;
    ComputeGoodKeyPnts(matches_good, keypnts_good_pre, keypnts_good_nxt);
    StoreGoodKeyPnts(keypnts_good_pre, keypnts_good_nxt);
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn  bool ComputeGoodKeyPnts
*
* \brief    get good key points from matches good
*
* \param[in]    matches_good --> target good matches
*
* \param[out]   keypnts_good_pre --> container for keypoints good in image at time 1
*               keypnts_good_nxt --> container for keypoints good in image at time 2
*
* \return   if seccessfult return true, otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool CVFeatureMatcher::
ComputeGoodKeyPnts(const std::vector<cv::DMatch> matches_good,
                   std::vector<cv::KeyPoint>& keypnts_good_pre,
                   std::vector<cv::KeyPoint>& keypnts_good_nxt)
{
    if(!_dofeaturematching_allready)
    {
        SfM_ERROR("Feature-Matching has been not be done, can not get keypnts");
        return false;
    }
    keypnts_good_pre.clear(); keypnts_good_nxt.clear();
    for (unsigned int i=0; i < matches_good.size(); i++)
    {
        keypnts_good_pre.push_back(_keypnts_pre[matches_good[i].queryIdx]);
        keypnts_good_nxt.push_back(_keypnts_nxt[matches_good[i].trainIdx]);
    }
    return true;
}
