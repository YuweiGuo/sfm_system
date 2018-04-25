/* ######################################################################### */
/* \file CVBundleAdjustor.h
 * \project name : SfMLib
 * \class name : CVBundleAdjustor
 *
 * \brief :     we do bundle adjustment in this class.
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */
#ifndef CVBUNDLEADJUSTOR_H
#define CVBUNDLEADJUSTOR_H

#include "BasicSetting.h"
#include "VirtualBundleAdjustor.h"

class CVBundleAdjustor : public VirtualBundleAdjustor
{
public:
    bool DoBundleAdjust();
public:
    bool Init(const cv::Matx33d camera_matx, const cv::Matx41d distcoeff,
              const std::vector<std::vector<CloudPoint> >& pnt_clouds,
              const std::vector<cv::Matx34d>& camera_poses,
              const int range = -1);

    bool GetImagepntsVSandPnt3D(const int64 pnts_num,
                                const unsigned int views_num,
                                const std::vector<std::vector<CloudPoint> >& pnt_clouds,
                                std::vector<std::vector<cv::Point2d> >& image_pnts,
                                std::vector<std::vector<int> >& objpnts_vs,
                                std::vector<cv::Point3d>& object_pnts,
                                const int range = -1); // -1: find all, others: deeps in the interval [i-1-range, i+2+range]

    bool GetNewPntCloudsAndCameraPoses(std::vector<std::vector<CloudPoint> >& pnt_clouds,
                                       std::vector<cv::Matx34d>& camera_poses);
private:
    cv::Matx33d _camera_matx;
    cv::Matx41d _distcoeff;
    std::vector<std::vector<CloudPoint> > _pnt_clouds;
    std::vector<cv::Matx34d> _camera_poses;
    int _range;
    bool _init_allready;
    bool _ba_allready;

public:
    CVBundleAdjustor(){
        _init_allready = false;
        _ba_allready = false;
        CLASS_CREATE_MSG("BundleAdjustor", true);
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


