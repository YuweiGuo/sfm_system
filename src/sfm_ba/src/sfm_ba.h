/* ######################################################################### */
/* \file sfm_ba.h
 * \project name : sfm_ba
 * \class name : SFM_BA
 * \brief :     wo make the bundle adjustment in the class SFM_BA with SfMLib
 *              the main jobs of it are :
 *              1). read point cloud and camera poses from SFM_MANAGER
 *              2). do bundle adjustement for the clouds and poses
 *              3). send out the completed point cloud
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */



#ifndef SFM_BA_H
#define SFM_BA_H

// std
#include <mutex>

// ros libs
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/message.h>

// opencv libs
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// self define
#include "setting.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

// sfm lib
#include <SfMTool.h>

// msg filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

class SFM_BA
{
public:
    /* ros node und spinner*/
    ros::NodeHandle _nh;
    ros::AsyncSpinner _spinner;

private:
    /* cloud synchronized receiver */
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                            sensor_msgs::PointCloud> Sync_Policy;

    message_filters::Subscriber<sensor_msgs::PointCloud>* _sfm_cloud_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* _sfm_pose_sub;

    message_filters::Synchronizer<Sync_Policy>* _sync;
    void SyncCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::PointCloudConstPtr& cloud_msg);

    /* ba cloud sender */
    ros::Publisher _sfm_ba_cloud_pub;
    void SfMBACloudSender();

private:
    /* point cloud , pose, leser recived */
    std::list<std::vector<CloudPoint> > _pnt_clouds_rv;
    std::list<cv::Matx34d> _camera_poses_rv;
    std::mutex _lock_rv;

    /* point cloud , pose, leser work now */
    std::vector<std::vector<CloudPoint> > _pnt_clouds_wk;
    std::vector<cv::Matx34d> _camera_poses_wk;

public:
    void Start();
    void Stop();
    void Run();
    bool DoSfMBA();
    void MsgLoop();
    void PoseStamped2Matx(const geometry_msgs::PoseStampedConstPtr& pose_msg, cv::Matx34d pose_matx);
    void GetPointCloudMsg(sensor_msgs::PointCloud& cloud_msg);

private:
    SfMTool _sfm_tool;

public:
    SFM_BA() : _spinner(0), _sfm_tool(cv::initModule_nonfree())
    {
        /* image reader and pnts_msg sender */
        {
            _sfm_cloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud>(_nh, SFM_MANAGER_CLOUD_NAME, 1000);
            _sfm_pose_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(_nh, SFM_MANAGER_CAMERA_POSE_NAME, 1000);
            _sync = new message_filters::Synchronizer<Sync_Policy>(Sync_Policy(10),
                                                                   *_sfm_pose_sub,
                                                                   *_sfm_cloud_sub);

            _sync->registerCallback(boost::bind(&SFM_BA::SyncCallback, this, _1, _2));

            _sfm_ba_cloud_pub = _nh.advertise<sensor_msgs::PointCloud>(SFM_BA_CLOUD_NAME, 100);
        }

        /* camera poses init for the default camera */
        {
            cv::Matx34d default_camera_pose(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
            _camera_poses_wk.push_back(default_camera_pose);
        }
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
