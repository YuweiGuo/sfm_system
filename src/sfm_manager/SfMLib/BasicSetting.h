/* ######################################################################### */
/* \file BasicSetting.h
 * \project name : SfMLib
 * \class name : HelpFunc
 * \brief : this class HelpFunc provides many help functions.
 *          they are all static functions. this means, we can call them easily from outside
 *          it includes:
 *          1). transform from cv::KeyPoint to cv::Points
 *          2). get pointcloud in cv from pointcloud in cloudpoint.
 *          3). print pointcloud on the screen
 *          4). 3*4 matrix to rotation and translation matrix
 *          5). rotation and translation matrix to 3*4 matrix
 *
 *          in addition, in this file we define here a important struct CloudPoint.
 *          and many control parameters
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */

#ifndef BASICSETTING_H
#define BASICSETTING_H
// SfMlib
#include "SfMLibObject.h"

// opencv libs
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>


#ifndef EPSILON
#define EPSILON 1e-6
#endif

#ifndef DEBUG
#define DEBUG 0
#endif

#ifndef DO_BA
#define DO_BA 1
#endif

#ifndef CLASS_CREATE_MSG
#define CLASS_CREATE_MSG(class_name, create_result) {\
    std::cout << "-----------------------" ;\
    std::cout << class_name;\
    if(create_result)\
        std::cout << " has been created";\
    else \
        std::cout << " could not create " ; \
    std::cout << "-----------------------" << std::endl ;\
}
#endif

#ifndef SfM_ERROR
#define SfM_ERROR(error_msg) {\
    std::cout << "SfM went error : " ; \
    std::cout << error_msg << " ." << std::endl;\
}
#endif

#ifndef SfM_SENDMSG
#define SfM_SENDMSG(msg) {\
    std::cout << msg << std::endl ; \
}
#endif

/* ------------------------------------------------------------------------- */
/*
 * struct CloudPoint
 *
 * \brief   a very important struct. it is the unit of our point cloud.
 *          the image points reference can help us to do SfM-Process, for example in pnp method
 *
 * \param   object_pnt --> 3D point in the real world
 *          pnt_colpr --> the color of object_pnt
 *          image_pnt_pre --> the reference of the 2D point on the image at time 1
 *          image_pnt_nxt --> the reference of the 2D point on the image at time 2
 *          error_pre --> the reprojection error of the 2D point on the image at time 1
 *          error_nxt --> the reprojection error of the 2D point on the image at time 2
*/
/* ------------------------------------------------------------------------- */
struct CloudPoint{
    cv::Point3d object_pnt; // pnt position
    cv::Vec3b pnt_color;    // pnt color
    cv::Point2d image_pnt_pre, image_pnt_nxt;   // corresponding points in 2 images
    double error_pre, error_nxt;    // reprojection error in 2 images
};

class HelpFunc : public SfMLibObject
{
public:
    /* default constructor */
    HelpFunc(){}
    /* ------------------------------------------------------------------------- */
    /* \fn bool KeyPntsToPnts
    *
    * \brief    transform the point type from std::vector<cv::KeyPoint> to std::vector<cv::Point2d>
    *
    * \param[in]    keypnts --> target point cloud in std::vector<cv::KeyPoint>
    *
    * \param[out]   pnts --> target point cloud in std::vector<cv::Point2d>
    *
    * \return if seccessful, return true. otherwise return false
    */
    /* ------------------------------------------------------------------------- */
    static bool KeyPntsToPnts(const std::vector<cv::KeyPoint>& keypnts, std::vector<cv::Point2d>& pnts)
    {
        if(keypnts.empty())
        {
            SfM_ERROR("keypnts is empty for function CVFeatureMatcher::KeyPntsToPnts");
            return false;
        }
        pnts.clear();
        for(int i = 0; i < keypnts.size(); i++)
        {
            pnts.push_back(keypnts[i].pt);
        }
        return true;
    }

    /* ------------------------------------------------------------------------- */
    /* \fn bool Get3DPointsFromPointCloud
    *
    * \brief    transform the point type from std::vector<CloudPoint> to std::vector<cv::Point3d>
    *
    * \param[in]    pnt_cloud --> target point cloud in std::vector<CloudPoint>
    *
    * \param[out]   pnts3d --> target point cloud in std::vector<cv::Point3d>
    *
    * \return if seccessful, return true. otherwise return false
    */
    /* ------------------------------------------------------------------------- */
    static bool Get3DPointsFromPointCloud(const std::vector<CloudPoint>& pnt_cloud, std::vector<cv::Point3d>& pnts3d)
    {
        if(pnt_cloud.empty())
        {
            SfM_ERROR("there is no pnt in pnt_cloud");
            return false;
        }
        pnts3d.clear();
        for(int i = 0; i < pnt_cloud.size(); i++)
        {
            pnts3d.push_back(pnt_cloud[i].object_pnt);
        }
        return true;
    }

    /* ------------------------------------------------------------------------- */
    /* \fn void PrintPointCloud
    *
    * \brief    print point cloud in std::vector<CloudPoint> on screen
    *
    * \param[in]    pnt_cloud --> target point cloud in std::vector<CloudPoint>
    */
    /* ------------------------------------------------------------------------- */
    static void PrintPointCloud(const std::vector<CloudPoint>& pnt_cloud)
    {
        if(pnt_cloud.empty())
        {
            SfM_ERROR("the pnt_cloud is empty, can not be printed");
            return;
        }
        SfM_SENDMSG("point cloud print : ");
        for(int i = 0; i < pnt_cloud.size(); i++)
        {
            SfM_SENDMSG("pnt"+std::to_string(i)+" : ("+
                           std::to_string(pnt_cloud[i].object_pnt.x)+","+
                           std::to_string(pnt_cloud[i].object_pnt.y)+","+
                           std::to_string(pnt_cloud[i].object_pnt.z)+") with color ("+
                           std::to_string(pnt_cloud[i].pnt_color.val[0])+","+
                           std::to_string(pnt_cloud[i].pnt_color.val[1])+","+
                           std::to_string(pnt_cloud[i].pnt_color.val[2])+")");
        }
    }

    /* ------------------------------------------------------------------------- */
    /* \fn bool Matx34ToRT
    *
    * \brief    transform 3*4 matrix to rotation and translation matrix
    *
    * \param[in]    camera_pose --> target 3*4 matrix
    *
    * \param[out]   rmatx --> target rotation matrix
    *               tmatx --> target translation matrix
    *
    * \return if seccessful, return true. otherwise return false
    */
    /* ------------------------------------------------------------------------- */
    static bool Matx34ToRT(const cv::Matx34d& camera_pose, cv::Matx33d &rmatx, cv::Matx31d& tmatx)
    {
        if(camera_pose == cv::Matx34d::zeros())
            return false;
        rmatx = cv::Matx33d(camera_pose(0,0), camera_pose(0,1), camera_pose(0,2),
                            camera_pose(1,0), camera_pose(1,1), camera_pose(1,2),
                            camera_pose(2,0), camera_pose(2,1), camera_pose(2,2));
        tmatx = cv::Matx31d(camera_pose(0,3), camera_pose(1,3), camera_pose(2,3));
        return true;
    }

    /* ------------------------------------------------------------------------- */
    /* \fn bool RTToMatx34
    *
    * \brief    transform  rotation and translation matrix to 3*4 matrix
    *
    * \param[in]   rmatx --> target rotation matrix
    *               tmatx --> target translation matrix
    *
    * \param[out]    camera_pose --> target 3*4 matrix
    *
    * \return if seccessful, return true. otherwise return false
    */
    /* ------------------------------------------------------------------------- */
    static bool RTToMatx34(const cv::Matx33d& rmatx, const cv::Matx31d& tmatx, cv::Matx34d& camera_pose)
    {
        if(rmatx == cv::Matx33d::zeros())
            return false;
        camera_pose = cv::Matx34d(rmatx(0,0), rmatx(0,1), rmatx(0,2), tmatx(0),
                                  rmatx(1,0), rmatx(1,1), rmatx(1,2), tmatx(1),
                                  rmatx(2,0), rmatx(2,1), rmatx(2,2), tmatx(2));
        return true;
    }

    /* ------------------------------------------------------------------------- */
    /* \fn std::vector<cv::Point3d> PointCloudToPoints
    *
    * \brief    transform the point type from std::vector<CloudPoint> to std::vector<cv::Point3d>
    *
    * \param[in]    pnt_cloud --> target point cloud in std::vector<CloudPoint>
    *
    * \return   target pointcloud in std::vector<cv::Point3d>
    */
    /* ------------------------------------------------------------------------- */
    static std::vector<cv::Point3d>
    PointCloudToPoints(const std::vector<CloudPoint>& pnt_cloud)
    {
        std::vector<cv::Point3d> pnts;
        for(int i = 0; i < pnt_cloud.size(); i++)
        {
            pnts.push_back(pnt_cloud[i].object_pnt);
        }
        return pnts;
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

