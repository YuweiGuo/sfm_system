/* ######################################################################### */
/* \file CVFeatureMatcher.h
 * \project name : SfMLib
 * \class name : CVFeatureMatcher
 *
 * \brief :     we do feature matching in this class.
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */
#ifndef CVFEATUREMATCHER_H
#define CVFEATUREMATCHER_H

#include "SfMLibFeatureMatcher.h"
#include "BasicSetting.h"

class CVFeatureMatcher : public SfMLibFeatureMatcher
{
public:
    CVFeatureMatcher();
    CVFeatureMatcher(const std::string FeatureDetecteMethod, const std::string DescriptorExtractorMethod);
public: // main functions
    bool Init(const cv::Mat image_pre, const cv::Mat image_nxt);
    bool DoFeatureMatching();
    bool ComputeGoodKeyPnts(const std::vector<cv::DMatch> matches_good,
                            std::vector<cv::KeyPoint>& keypnts_good_pre,
                            std::vector<cv::KeyPoint>& keypnts_good_nxt);
private:
    cv::Mat _image_pre, _image_nxt;
    std::vector<cv::KeyPoint> _keypnts_pre, _keypnts_nxt;
    cv::Mat _descriptor_pre, _descriptor_nxt;
    std::vector<cv::DMatch> _matches;
    std::string _detector_method, _extractor_method;

private:
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _extractor;
    bool _init_allready;
    bool _dofeaturematching_allready;
};

#endif /* _AUTHOR_FILENAME_H */
/* ------------------------------------------------------------------------- */
/*
 * RIGHT OF USE. This document may neither be passed on to third parties or
 * reproduced nor its contents utilized or divulged without the expressed
 * prior permission of the Institute for Real-Time Learning Systems. In case of
 * contravention, the offender shall be liable for damages.
 *
 *
 * COPYRIGHT (C) Institut für Echtzeit Lernsysteme, Prof. K.-D. Kuhnert 2007-2015
 *
 */
/* ------------------------------------------------------------------------- */


