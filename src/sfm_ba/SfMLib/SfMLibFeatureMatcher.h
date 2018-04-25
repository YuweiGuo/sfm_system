/* ######################################################################### */
/* \file SfMLibFeatureMatcher.h
 * \project name : SfMLib
 * \class name : SfMLibFeatureMatcher
 * \brief :     a implementation for virtual class.
 *              hier we define the output parameters for the class.
 *              and the corresponding operator.
 *              the virtual function is not implemented jet.
 *
 *              in this class we defined :
 *              1). output parameter _keypnts_good_pre
 *              2). output parameter _keypnts_good_nxt
 *              3). output parameter _matches_good
 *              4). the method to get matches and keypnts GetGoodMachtes, GetGoodKeyPnts
 *              5). the method to store matches and keypnts StoreGoodMachtes, StoreGoodKeyPnts
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */
#ifndef SFMLIBFEATUREMATCHER_H
#define SFMLIBFEATUREMATCHER_H

#include "BasicSetting.h"
#include "VirtualFeatureMatcher.h"

class SfMLibFeatureMatcher : public VirtualFeatureMatcher
{
public:
    SfMLibFeatureMatcher(){}
    virtual bool DoFeatureMatching() = 0;
private:
    std::vector<cv::KeyPoint> _keypnts_good_pre, _keypnts_good_nxt;
    std::vector<cv::DMatch> _matches_good;
public:
    bool GetGoodMachtes(std::vector<cv::DMatch>& matches_good);
    bool GetGoodKeyPnts(std::vector<cv::KeyPoint>& keypnts_good_pre, std::vector<cv::KeyPoint>& keypnts_good_nxt);
public:
    void StoreGoodMachtes(const std::vector<cv::DMatch>& matches_good);
    void StoreGoodKeyPnts(const std::vector<cv::KeyPoint>& keypnts_good_pre, const std::vector<cv::KeyPoint>& keypnts_good_nxt);
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

