/* ######################################################################### */
/* \file viewer.h
 * \project name : sfm_viewer
 * \class name : VIEWER
 * \brief : the viwer receives point cloud from sfm_manager, sfm_ba and other pointcloud sender(laser).
 *          then it shows them with pcl visualizator.
 *          furthermore it provides many compare functions to determine the degree of closeness of 2 pointcloud
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */

#ifndef VIEWER_H
#define VIEWER_H

/* standart libs*/
#include <mutex>

/* ros und pcl*/
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/* opencv */
#include <opencv2/opencv.hpp>

/* msg type von sfm */
#include "setting.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


class VIEWER
{
public:
    /* run programm */
    void run();
private:
    /* start ros loop */
    void start();
    /* stop ros loop */
    void stop();
    /* show clouds */
    void showCloud();
private:
    /* cloud update control */
    bool _manager_update;
    std::mutex _lock;
private:
    const std::string _cloudName = "point_cloud";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _sfm_cloud;      // point cloud for sfm_manager for store and compute
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _laser_cloud;    // point cloud for laserscanner for store and compute
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _ba_cloud;       // point cloud for sfm_ba or other nodes, which ónly wants to show, for store and compute
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _all_cloud;      // the point cloud pool for show. other pointcloud, who want to show itself, must add to this pool.
private:
    void SetCameraPose(const pcl::PointXYZ position, const pcl::PointXYZ direction, const pcl::PointXYZ up);    // for camera control

private:
    /* ros node and spinner */
    ros::NodeHandle _nh;
    ros::AsyncSpinner _spinner;

private:
    /* receive msg , subs for sfm_manager */
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                            geometry_msgs::PoseStamped,
                                                            sensor_msgs::PointCloud> Manager_Sync_Policy;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* _sfm_camera_pose_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* _sfm_amor_pose_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud>* _sfm_manager_cloud_sub;

    message_filters::Synchronizer<Manager_Sync_Policy>* _sfm_sync;
    void SfMManagerSyncCallback(const geometry_msgs::PoseStampedConstPtr& camera_pose_msg,
                                const geometry_msgs::PoseStampedConstPtr& amor_pose_msg,
                                const sensor_msgs::PointCloudConstPtr& cloud_msg);
private:
    /* receive msg , subs for laser */
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                            sensor_msgs::PointCloud> laser_Sync_Policy;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* _amor_pose_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud>* _laser_cloud_sub;

    message_filters::Synchronizer<laser_Sync_Policy>* _laser_sync;
    void laserSyncCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg,
                           const sensor_msgs::PointCloudConstPtr& cloud_msg);
private:
    /* receive msg , subs for ba */
    ros::Subscriber _ba_cloud_sub;
    void BACallback(const sensor_msgs::PointCloudConstPtr& cloud_msg);
private:
    /* point distance */
    float _points_distance;
    /* compare matrix */
    bool _compare_matx_computet;
    cv::Matx44d _compare_matx;
    float _compare_error;
    /* more compare functions */
    bool DoCompare();
    bool ComputeCompareMatx();
    bool CheckMeetOrNot(const cv::Point3d &sfm_cloud_pnt, const std::vector<int>&  laser_pnts_box, const float& error);
    std::vector<cv::Matx31d> _camera_positions_4comp;
    std::vector<cv::Matx31d> _amor_positions_4comp;
    std::vector<std::vector<std::vector<std::vector<int> > > > _laser_pnts_box;

    bool AutomaticParameterBasedCompare();
    bool AutomaticParameterBasedAngleCompare();

private:
    /* for the cloud show */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;   // pcl visualizer

private:
    /* filters */
    bool MakeBundingBoxFilter(std::vector<pcl::PointXYZRGB>& point_cloud);
    bool _USE_BUNDINGBOX_FILTER;
    float _bbox_radius;

private: // for control
    void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void);      // for the keyboard interactive
    bool sfm_cloud_show;
    bool laser_cloud_show;
    bool ba_cloud_show;
    bool sfm_laser_pose_show;
    bool sfm_camera_pose_show;
    bool use_compare_matx;
    bool show_coordinate;

    bool transform;
    int x_change, y_change, z_change;
    int change_rat;

public:
    VIEWER() : _spinner(0)
    {
        /* viewer and cloud continer init */
        {
            _viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("sfm 3D Viewer"));
            _sfm_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            _laser_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            _all_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            _ba_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        }
        /* manager msg receiver */
        {
            /* for laser */
            _laser_cloud_sub =
                    new message_filters::Subscriber<sensor_msgs::PointCloud>(_nh, AMOR_LASER_POINT_CLOUD_NODE_NAME, 10);
            _amor_pose_sub =
                    new message_filters::Subscriber<geometry_msgs::PoseStamped>(_nh, AMOR_POSE_NODE_NAME, 10);

            _laser_sync = new message_filters::Synchronizer<laser_Sync_Policy>(laser_Sync_Policy(10),
                                                                   *_amor_pose_sub,
                                                                   *_laser_cloud_sub);
            _laser_sync->registerCallback(boost::bind(&VIEWER::laserSyncCallback, this, _1, _2));

            /* for sfm */
            _sfm_manager_cloud_sub =
                    new message_filters::Subscriber<sensor_msgs::PointCloud>(_nh, SFM_MANAGER_CLOUD_NAME, 10);
            _sfm_camera_pose_sub =
                    new message_filters::Subscriber<geometry_msgs::PoseStamped>(_nh, SFM_MANAGER_CAMERA_POSE_NAME, 10);
            _sfm_amor_pose_sub =
                    new message_filters::Subscriber<geometry_msgs::PoseStamped>(_nh, SFM_MANAGER_AMOR_POSE_NAME, 10);

            _sfm_sync = new message_filters::Synchronizer<Manager_Sync_Policy>(Manager_Sync_Policy(10),
                                                                       *_sfm_camera_pose_sub,
                                                                       *_sfm_amor_pose_sub,
                                                                       *_sfm_manager_cloud_sub);
            _sfm_sync->registerCallback(boost::bind(&VIEWER::SfMManagerSyncCallback, this, _1, _2, _3));

            /* for ba */
            _ba_cloud_sub = _nh.subscribe<sensor_msgs::PointCloud>(SFM_BA_CLOUD_NAME, 100, &VIEWER::BACallback, this);
        }
        /* filter */
        {
            _USE_BUNDINGBOX_FILTER = false;
            _bbox_radius = 0.4;
        }
        /* for compare */
        {
            _points_distance = 0.05;
            _compare_matx = cv::Matx44d::eye();
            _compare_matx_computet = false;
            _compare_error = 0;
            _laser_pnts_box =
                    std::vector<std::vector<std::vector<std::vector<int> > > >(101,
                    std::vector<std::vector<std::vector<int> > >(100,
                    std::vector<std::vector<int> >(100,
                    std::vector<int>())));
        }
        /* for control */
        {
            show_coordinate = false;
            sfm_cloud_show = true;
            laser_cloud_show = true;
            ba_cloud_show = true;
            sfm_laser_pose_show = true;
            sfm_camera_pose_show = true;
            use_compare_matx = false;

            transform = false;
            x_change = 0,y_change = 0,z_change = 0;
            change_rat = 2;
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

