/* ######################################################################### */
/* \file EPGCameraPoseComputer.h
 * \project name : SfMLib
 * \class name : EPGCameraPoseComputer
 * \brief :     this class is a subclass of SfMLibCameraPoseComputer
 *              the main job of this class is that use the epipolar geometry method to compute camera pose
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */

#ifndef EPGCAMERAPOSECOMPUTER_H
#define EPGCAMERAPOSECOMPUTER_H

#include "BasicSetting.h"
#include "VirtualCameraPoseComputer.h"
#include "SfMLibCameraPoseComputer.h"
#include "LSTriangulator.h"
#include "CVFeatureMatcher.h"

class EPGCameraPoseComputer : public SfMLibCameraPoseComputer
{
public:
    bool DoCameraPoseCompute();
public: // public interface
    bool Init(const std::vector<cv::KeyPoint> keypnts_good_pre,
              const std::vector<cv::KeyPoint> keypnts_good_nxt,
              const std::vector<cv::DMatch> matches_good,
              const cv::Matx33d camera_matx,
              const cv::Matx41d distcoeff,
              const float& epg_firstlength = 1.0);
private:
    void TakeSVDOfEMatx(const cv::Mat_<double> EMatx,
                        cv::Mat_<double>& svd_u,
                        cv::Mat_<double>& svd_vt,
                        cv::Mat_<double>& svd_w);
    bool DecomposeEMatxtoRandT(const cv::Mat_<double> EMatx,
                               cv::Mat_<double>& R1, cv::Mat_<double>& R2,
                               cv::Mat_<double>& T1, cv::Mat_<double>& T2);
    bool CheckCoherentRotation(const cv::Mat_<double> R);
    bool TestTriangulation(const std::vector<CloudPoint> pnt_cloud, const cv::Matx34d pose, std::vector<uchar>& status);

private:
    std::vector<cv::KeyPoint> _keypnts_good_pre, _keypnts_good_nxt;
    cv::Matx33d _camera_matx; cv::Matx41d _distcoeff;
    std::vector<cv::DMatch> _matches_good;
    float _epg_firstlength;
    bool _init_allready;
    bool _compute_allready;

public:
    EPGCameraPoseComputer(){
        _init_allready = false;
        _init_allready = false;
        _compute_allready = false;
        CLASS_CREATE_MSG("EPGCameraPoseComputer", true);
    }


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


