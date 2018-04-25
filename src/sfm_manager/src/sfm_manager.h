/* ######################################################################### */
/* \file sfm_manager.h
 * \project name : sfm_manager
 * \class name : SFM_MANAGER
 * \brief : the SfMManger controls all SfM-Process, make 2D infromation to 3D points with SfMLib
 *          main jobs are :
 *          0). sfm-loop control
 *          1). read images und poses from ROS
 *          2). run SfMProcess for 2 images
 *          3). error control
 *          4). send out pointcloud
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */


#ifndef SFM_MANAGER_H
#define SFM_MANAGER_H

// standart libs
#include <vector>
#include <list>
#include <mutex>

// self define
#include "setting.h"

// opencv libs
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// ros libs
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/message.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

// sfmlib
#include <SfMTool.h>

class SFM_MANAGER
{
public:
    /* ros node and spinner*/
    ros::NodeHandle _nh;
    ros::AsyncSpinner _spinner;
private:
    /* update */
    int64 _read_image_steps;
    float _READSTEPDISTANCE;

    int _DOSfMDISTANCE;

private:
    /* amor image receiver */
    message_filters::Subscriber<sensor_msgs::Image>* _amor_image_sub;
    /* Images from lesener */
    std::list<cv::Mat> _image_list;

    /* sfm manager amor poses receiver */
    message_filters::Subscriber<geometry_msgs::PoseStamped>* _amor_pose_sub;
    /* pose from amor/pose */
    std::list<cv::Matx34d> _amor_pose_list;

private:
    /* msg filter for msg synchronization */
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                            sensor_msgs::Image> Sync_Policy;
    message_filters::Synchronizer<Sync_Policy>* _sync;
    void SyncCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg,
                      const sensor_msgs::ImageConstPtr& image_msg);

private:
    /* sfm manager cloud sender */
    int _send_id;
    ros::Publisher _sfm_pntcloud_pub;
    ros::Publisher _sfm_camera_pose_pub;
    ros::Publisher _sfm_amor_pose_pub;
    bool SfMManagerCloudPoseSender(const std::vector<CloudPoint> pnt_cloud,
                               const cv::Matx34d camera_pose,
                               const cv::Matx34d amor_pose);

private:
    /* for false handle */
    int _block_size = 3;
    std::list<std::vector<CloudPoint> > _pnt_cloud_pre_list;
    std::list<cv::Mat> _image_pre_list;
    std::list<cv::Matx34d> _camera_pose_pre_list;
    cv::Matx34d _amor_pose_pre;

    /* error control */
    float _error_of_epg_step;   // max error in every normal step
    float _error_of_pnp_step;
    float _error_of_bads;   // max error in the bad step

private:
    /* sfm tool */
    SfMTool _sfm_tool;

public:
    /* run the program*/
    bool Run();

private:
    /* sfm start */
    bool Start();
    /* sfm stop */
    bool Stop();

private:
    /* do sfm */
    bool SfMProcess();

private:
    /* help function */
    bool CVPointCloudToSensorPointCloudMsg(const std::vector<CloudPoint> sfm_pnt_cloud, const int id, const ros::Time stamp, sensor_msgs::PointCloud& pnt_cloud_msg);
    bool CVPoseToGeometryPoseMsg(const cv::Matx34d camera_pose, const int id, const ros::Time stamp, geometry_msgs::PoseStamped& pose_msg);
    bool Matx34ToRT(const cv::Matx34d camera_pose, cv::Matx33d& rmatx, cv::Matx31d& tmatx);
    cv::Matx34d PoseStampedToCVMatx(const geometry_msgs::PoseStamped pose_msg);

private:
    /* point cloud filter setting and functions */
    bool _USE_CLOUDFILTER;
    std::vector<int> PointCloudFilter(const std::vector<CloudPoint> pnt_cloud,
                                      const cv::Matx34d camera_pose_pre,
                                      const cv::Matx34d camera_pose_nxt);
    bool _USE_BUNDINGBOX;
    float _bbox_radius;
    std::vector<int> BundingBoxFilter(const std::vector<CloudPoint> pnt_cloud);

private:
    /* for other image node */
    ros::Subscriber _image_node_sub;
    ros::Publisher _image_node_pub;
    void ImageNodeCallback(const sensor_msgs::ImageConstPtr& image_msg);
    bool SfMManagerCloudSender(const std::vector<CloudPoint>& sfm_pntcloud);

public:
    /* Constructor */
    SFM_MANAGER() : _spinner(0), _sfm_tool(
#if USE_FASTORB_METHOD
                                     "PyramidFAST", "ORB",
#endif
                                     cv::initModule_nonfree())
    {   
        /* receiver and sender */
        {
            _read_image_steps = -1;

            _amor_image_sub =
                    new message_filters::Subscriber<sensor_msgs::Image>(_nh, AMOR_IMG_NODE_NAME, 1000);
            _amor_pose_sub =
                    new message_filters::Subscriber<geometry_msgs::PoseStamped>(_nh, AMOR_POSE_NODE_NAME, 1000);

            _sync = new message_filters::Synchronizer<Sync_Policy>(Sync_Policy(10),
                                                                   *_amor_pose_sub,
                                                                   *_amor_image_sub);
#if FOR_IMG_AND_POSE
            _sync->registerCallback(boost::bind(&SFM_MANAGER::SyncCallback, this, _1, _2));
#endif
            _send_id = 0;
            _sfm_pntcloud_pub = _nh.advertise<sensor_msgs::PointCloud>(SFM_MANAGER_CLOUD_NAME, 100);
            _sfm_camera_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>(SFM_MANAGER_CAMERA_POSE_NAME, 100);
            _sfm_amor_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>(SFM_MANAGER_AMOR_POSE_NAME, 100);
        }

        /* init sfmtool */
        {
            _sfm_tool.Init(camera_matx, dist_coeff);
        }

        /* sfm step control */
        {
            _READSTEPDISTANCE = 0.2;
            _DOSfMDISTANCE = 4;
        }
        /* filter setting */
        {
            _bbox_radius = 0.6;
            _USE_BUNDINGBOX = true;
            _USE_CLOUDFILTER = true;
        }

        /* for image node */
        {
#if FOR_IMG_AND_POSE
#else
            _image_node_sub = _nh.subscribe<sensor_msgs::Image>(OTHER_IMAGE_NODE, 100, &SFM_MANAGER::ImageNodeCallback, this);
#endif
            _image_node_pub = _nh.advertise<sensor_msgs::PointCloud>(SFM_BA_CLOUD_NAME, 100);
        }

        /* error control */
        {
            _error_of_epg_step = 0.8;
            _error_of_pnp_step = 1.0;
            _error_of_bads = 30;
        }
    }
};


#endif/* _AUTHOR_FILENAME_H */
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
