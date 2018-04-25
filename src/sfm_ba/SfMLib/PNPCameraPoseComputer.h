/* ######################################################################### */
/* \file PNPCameraPoseComputer.h
 * \project name : SfMLib
 * \class name : PNPCameraPoseComputer
 * \brief :     this class is a subclass of SfMLibCameraPoseComputer
 *              the main job of this class is that use the cv::pnp method to compute camera pose
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */

#ifndef PNPCAMERAPOSECOMPUTER_H
#define PNPCAMERAPOSECOMPUTER_H

#include "BasicSetting.h"
#include "SfMLibCameraPoseComputer.h"

class PNPCameraPoseComputer : public SfMLibCameraPoseComputer
{
public:
    bool DoCameraPoseCompute();
public:
    bool Init(const std::vector<cv::KeyPoint> keypnts_good_pre,
              const std::vector<cv::KeyPoint> keypnts_good_nxt,
              const std::vector<cv::DMatch> matches_good,
              const cv::Matx33d camera_matx,
              const cv::Matx41d distcoeff,
              const std::vector<CloudPoint>& pnt_cloud_preframe);
private:
    unsigned int FindSame2D3DPoints(const std::vector<cv::KeyPoint> keypnts_new_pre,
                                    const std::vector<cv::KeyPoint> keypnts_new_nxt,
                                    const std::vector<CloudPoint> pnt_cloud_preframe,
                                    std::vector<cv::Point2d>& pnts2d_same_nxt,
                                    std::vector<cv::Point3d>& pnts3d_same_preframe);

    bool CameraPosPnPRansac(const std::vector<cv::KeyPoint> keypnts_new_pre,
                            const std::vector<cv::KeyPoint> keypnts_new_nxt,
                            const std::vector<CloudPoint> pnt_cloud_preframe,
                            const cv::Matx33d camera_matx,
                            const cv::Matx41d distcoeff,
                            cv::Matx34d& camera_pos);
private:
    std::vector<CloudPoint> _pnt_cloud_preframe;
private:
    std::vector<cv::KeyPoint> _keypnts_good_pre, _keypnts_good_nxt;
    cv::Matx33d _camera_matx; cv::Matx41d _distcoeff;
    std::vector<cv::DMatch> _matches_good;
private:
    bool _init_allready;

public:
    PNPCameraPoseComputer(){
        _init_allready = false;
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


