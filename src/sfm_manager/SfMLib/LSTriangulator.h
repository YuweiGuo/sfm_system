/* ######################################################################### */
/* \file LSTriangulator.h
 * \project name : SfMLib
 * \class name : LSTriangulator
 *
 * \brief :     we do triangulation in this class.
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */

#ifndef LSTRIANGULATOR_H
#define LSTRIANGULATOR_H

#include "BasicSetting.h"
#include "SfMLibTriangulator.h"

class LSTriangulator : public SfMLibTriangulator
{
public:
    bool Init(const cv::Mat image_pre, const cv::Mat image_nxt,
              const std::vector<cv::KeyPoint>& keypnts_new_pre, const std::vector<cv::KeyPoint>& keypnts_new_nxt,
              const cv::Matx33d& camera_matx, const cv::Matx34d& camera_pose_pre, const cv::Matx34d& camera_pose_nxt);
    bool UpdateImage(const cv::Mat image_pre, const cv::Mat image_nxt);

public:
    double DoTriangulation();

private:    // help functions
    cv::Mat_<double> LinearLSTriangulation(const cv::Point3d image_pnt_pre, const cv::Matx34d camera_pose_pre,
                                           const cv::Point3d image_pnt_nxt, const cv::Matx34d camera_pose_nxt);
    cv::Mat_<double> IterativeLinearLSTriangulation(const cv::Point3d image_pnt_pre, const cv::Matx34d camera_pose_pre,
                                                    const cv::Point3d image_pnt_nxt, const cv::Matx34d camera_pose_nxt);
private:
    std::vector<cv::KeyPoint> _keypnts_new_pre, _keypnts_new_nxt;
    cv::Matx33d _camera_matx;
    cv::Matx34d _camera_pose_pre, _camera_pose_nxt;

private:
    cv::Mat _image_pre, _image_nxt;
    bool _init_allready;
    bool _triangulation_allready;

public:
    LSTriangulator(){
        _init_allready = false;
        _triangulation_allready = false;
        CLASS_CREATE_MSG("Triangulator", true);
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


