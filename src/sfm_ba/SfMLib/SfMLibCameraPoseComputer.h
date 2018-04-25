/* ######################################################################### */
/* \file SfMLibCameraPoseComputer.h
 * \project name : SfMLib
 * \class name : SfMLibCameraPoseComputer
 * \brief :     a implementation for virtual class.
 *              hier we define the output parameters for the class.
 *              and the corresponding operator.
 *              the virtual function is not implemented jet.
 *
 *              in this class we defined :
 *              1). output parameter _keypnts_new_pre
 *              2). output parameter _keypnts_new_nxt
 *              3). output parameter _matches_new
 *              4). output parameter _camera_pose_nxt
 *              5). the method to get matches and keypnts GetMachtesNew, GetKeyPntsNew
 *              6). the method to store matches and keypnts StoreKeyPntsMatchesNew
 *              7). the method to get _camera_pose_nxt GetCameraPose
 *              8). the method to store pose_nxt StoreCameraPoseNxt
 *
 *              this class is for the 2 subclasses epg and pnp.
 *              the functions, who is needed by thist 2 class, will be defined here too.(ComputeFundamentalMatx)
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */
#ifndef SFMLIBCAMERAPOSECOMPUTER_H
#define SFMLIBCAMERAPOSECOMPUTER_H


#include "BasicSetting.h"
#include "VirtualCameraPoseComputer.h"

class SfMLibCameraPoseComputer : VirtualCameraPoseComputer
{
public:
    SfMLibCameraPoseComputer(){}
public:
    virtual bool DoCameraPoseCompute() = 0;
public: // all required function
    cv::Mat_<double> ComputeFundamentalMatx(const std::vector<cv::KeyPoint> keypnts_good_pre,
                                            const std::vector<cv::KeyPoint> keypnts_good_nxt,
                                            const std::vector<cv::DMatch> matches_good,
                                            std::vector<cv::KeyPoint>& pnts_new_pre,
                                            std::vector<cv::KeyPoint>& pnts_new_nxt,
                                            std::vector<cv::DMatch>& matches_new);
public: // data operation function
    bool GetCameraPose(cv::Matx34d& camera_pose);
    bool GetKeyPntsNew(std::vector<cv::KeyPoint>& keypnts_new_pre,
                       std::vector<cv::KeyPoint>& keypnts_new_nxt);
    bool GetMatchesNew(std::vector<cv::DMatch>& matches_new);
    void StoreKeyPntsMatchesNew(const std::vector<cv::KeyPoint> pnts_new_pre,
                                const std::vector<cv::KeyPoint> pnts_new_nxt,
                                const std::vector<cv::DMatch> matches_new);
    void StoreCameraPoseNxt(const cv::Matx34d camera_pose_nxt);

private: // out data
    std::vector<cv::KeyPoint> _keypnts_new_pre, _keypnts_new_nxt;
    std::vector<cv::DMatch> _matches_new;
    cv::Matx34d _camera_pose_nxt;
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

