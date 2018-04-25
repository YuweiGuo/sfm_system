/* ######################################################################### */
/* \file SfMTool.h
 * \project name : SfMLib
 * \class name : SfMTool
 * \brief : the class uses SfMLib , provides 3 methode to do SfM for 2 images.
 *          it includes :
 *          1). do sfm with epg method.
 *          2). do sfm with pnp method.
 *          3). get camera positions from other resources, then do sfm.
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */
#ifndef SFMTOOL_H
#define SFMTOOL_H

#include "SfMLibObject.h"
#include "CVBundleAdjustor.h"
#include "CVFeatureMatcher.h"
#include "EPGCameraPoseComputer.h"
#include "LSTriangulator.h"
#include "PNPCameraPoseComputer.h"

class SfMTool : public SfMLibObject
{
public:
    /* constructors */
    SfMTool(bool opencv_nonfree_init);
    SfMTool(std::string FeatureDetecteMethod, std::string DescriptorExtractorMethod, bool opencv_nonfree_init);
public:
    /* init parameters for tool */
    bool Init(const cv::Matx33d camera_matx, const cv::Matx41d distcoeff);
    /* 3 methodes to do sfm */
    bool DoSfM42ImagesWithEPG(const cv::Mat image_pre,
                              const cv::Mat image_nxt,
                              const cv::Matx34d& camera_pose_pre,
                              std::vector<CloudPoint>& pnt_cloud_now,
                              cv::Matx34d& camera_pose_now,
                              float& error,
                              const float epg_firstlength = 1.0);

    bool DoSfM42ImagesWithPoses(const cv::Mat image_pre, const cv::Matx34d camera_pose_pre,
                                const cv::Mat image_nxt, const cv::Matx34d camera_pose_nxt,
                                std::vector<CloudPoint>& pnt_cloud_now,
                                float& error);

    bool DoSfM4ImagesWithPNP(const cv::Mat image_pre,
                             const cv::Mat image_nxt,
                             const cv::Matx34d& camera_pose_pre,
                             const std::vector<CloudPoint>& pnt_cloud_pre,
                             std::vector<CloudPoint>& pnt_cloud_now,
                             cv::Matx34d& camera_pose_now,
                             float& error);
private:
    /* componentes from SfMLib */
    CVFeatureMatcher _feature_matcher;
    EPGCameraPoseComputer _epg_camera_pose_computer;
    PNPCameraPoseComputer _pnp_camera_pose_computer;
    LSTriangulator _triangulator;
public:
    CVBundleAdjustor _bundleadjustor;
private:
    /* camera parameter setting */
    cv::Matx33d _camera_matx;
    cv::Matx41d _distcoeff;
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

