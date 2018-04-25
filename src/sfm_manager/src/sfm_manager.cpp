#include "sfm_manager.h"

/* ------------------------------------------------------------------------- */
/* \fn bool Run()
*
* \brief for the outside ros loop call
*/
/* ------------------------------------------------------------------------- */
bool SFM_MANAGER::
Run()
{
    Start();
    Stop();
}

/* ------------------------------------------------------------------------- */
/* \fn bool Start()
*
* \brief start the ros loop, and run SfM-Process
*/
/* ------------------------------------------------------------------------- */
bool SFM_MANAGER::
Start()
{
    std::cout << "sfm manager is start..." << std::endl;
    /* starten die ros schleife */
    _spinner.start();
    /* start sfm */
    SfMProcess();
}

/* ------------------------------------------------------------------------- */
/* \fn bool Stop()
*
* \brief termination ros loop
*/
/* ------------------------------------------------------------------------- */
bool SFM_MANAGER::
Stop()
{
    /* stoppen die ros schleife */
    _spinner.stop();
}

/* ------------------------------------------------------------------------- */
/* \fn bool SfMProcess
*
* \brief this function maintains a SfMProzess cycle,
*        in this cycle it read images (and poses of cameras),
*        then uses them to compute 3D pointcloud. (details in my thesis)
*
*/
/* ------------------------------------------------------------------------- */
bool SFM_MANAGER::
SfMProcess()
{
    int64 step = -1;
    while(ros::ok())
    {
        if(_image_list.size() < _DOSfMDISTANCE
#if FOR_IMG_AND_POSE
                || _amor_pose_list.size() < _DOSfMDISTANCE
#endif
                ) continue;
        step ++;

        /* define image_nxt and amor_pose_now */
        cv::Mat image_nxt;
#if FOR_IMG_AND_POSE
        cv::Matx34d amor_pose_now;
#endif
        /* for 0 step store the default data */
        if(step == 0)
        {
            /* get important image nxt */
            image_nxt = _image_list.front();
            {
                image_nxt = _image_list.front();
                _image_list.pop_front();
            }
#if FOR_IMG_AND_POSE
            /* get the amor pose */
            {
                amor_pose_now = _amor_pose_list.front();
                _amor_pose_list.pop_front();
            }
#endif
            _pnt_cloud_pre_list.push_front(std::vector<CloudPoint>());
            _image_pre_list.push_front(image_nxt);
            _camera_pose_pre_list.push_front(cv::Matx34d(1, 0, 0, 0,
                                                         0, 1, 0, 0,
                                                         0, 0, 1, 0));
#if FOR_IMG_AND_POSE
            _amor_pose_pre = amor_pose_now;
#endif
            std::cout << "step " << step << " is completed ! " << std::endl;
            continue;
        }

        /* init image nxt and pose for step >= 1 */
        std::vector<cv::Mat> image_wklist;
#if FOR_IMG_AND_POSE
        std::vector<cv::Matx34d> amor_pose_wklist;
#endif
        {
            for(int i = 0; i < _DOSfMDISTANCE; i++)
            {
                cv::Mat img_tmp = _image_list.front();
                image_wklist.push_back(img_tmp);
                _image_list.pop_front();
#if FOR_IMG_AND_POSE
                cv::Matx34d pose_tmp = _amor_pose_list.front();
                amor_pose_wklist.push_back(pose_tmp);
                _amor_pose_list.pop_front();
#endif
            }
        }

        /* define wanted cloud und camera pose and then compute */
        std::vector<CloudPoint> pnt_cloud_now; cv::Matx34d camera_pose_now;
        bool SfM_Done = false;

        /* rt storage define , for sfm_done == false situation */
        std::vector<cv::Mat> image_rt;
        std::vector<std::vector<CloudPoint> > pnt_cloud_rt;
        std::vector<cv::Matx34d> camera_pose_rt;
        std::vector<float> tri_error_rt;
        std::vector<int> id_rt;

        /* begin */
        for( int id_wk = _DOSfMDISTANCE-1; id_wk >= 0 && !SfM_Done; --id_wk)
        {
            /* get work source */
            {
                /* get important image nxt */
                image_nxt = image_wklist[id_wk];
#if FOR_IMG_AND_POSE
                /* get the amor pose */
                amor_pose_now = amor_pose_wklist[id_wk];
#endif
            }
            if(step == 1)
            {
                /* compute sfm */
                {
                    bool rt = false;    float tri_error = 0;
#if FOR_IMG_AND_POSE
                    float epg_firstlength = cv::norm((_amor_pose_pre-amor_pose_now)*cv::Matx41d(0,0,0,1));
#endif
                    rt = _sfm_tool.DoSfM42ImagesWithEPG(_image_pre_list.front(), image_nxt, _camera_pose_pre_list.front(),
                                                        pnt_cloud_now, camera_pose_now, tri_error,
#if FOR_IMG_AND_POSE
                                                        epg_firstlength
#else
                                                        1
#endif
                                                        );
                    if( rt == false )   // total false
                    {
                        SfM_Done = false;
                    }
                    else
                    if( tri_error < _error_of_epg_step)    // super good
                    {
#if FOR_IMG_AND_POSE
                        std::cout <<"first step is seccesfull with epg_firstlength  : " << epg_firstlength << std::endl;
#endif
                        /* break the wk loop */
                        SfM_Done = true;

                        /* push back unused image and image */
                        for(int id_wk_nxt = _DOSfMDISTANCE-1 ; id_wk_nxt > id_wk ; --id_wk_nxt)
                        {
                            _image_list.push_front(image_wklist[id_wk_nxt]);
#if FOR_IMG_AND_POSE
                            _amor_pose_list.push_front(amor_pose_wklist[id_wk_nxt]);
#endif
                        }
                        /* break the current loop */
                        break;
                    }
                    else    // store not good result
                    {
                        SfM_Done = false;
                    }
                }
            }
            else   // for others we use PnP method
            {
                /* compute sfm */
                {
                    std::list<cv::Mat>::iterator it_image = _image_pre_list.begin();
                    std::list<std::vector<CloudPoint> >::iterator it_cloud = _pnt_cloud_pre_list.begin();
                    std::list<cv::Matx34d>::iterator it_pose_pre = _camera_pose_pre_list.begin();
                    int id_pre = 0;

                    bool rt = false; float tri_error = 0;
                    for(;it_image != _image_pre_list.end() && !SfM_Done; ++it_image, ++it_cloud, ++it_pose_pre, ++id_pre)
                    {
                        std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                        std::cout << "do sfm for [wk, pre] ---> [" << id_wk << " , " << id_pre << " ]" << std::endl;
                        std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                        rt = _sfm_tool.DoSfM4ImagesWithPNP(*it_image, image_nxt, *it_pose_pre, *it_cloud,
                                                         pnt_cloud_now, camera_pose_now, tri_error);
                        if( rt == false )   // total false
                        {
                            SfM_Done = false;
                        }
                        else
                        {
                            if( tri_error < _error_of_pnp_step )  // super good
                            {
                                /* break the wk loop */
                                SfM_Done = true;
                                /* push back unused image and image */

                                for(int id_wk_nxt = _DOSfMDISTANCE-1 ; id_wk_nxt > id_wk ; --id_wk_nxt)
                                {
                                    _image_list.push_front(image_wklist[id_wk_nxt]);
#if FOR_IMG_AND_POSE
                                    _amor_pose_list.push_front(amor_pose_wklist[id_wk_nxt]);
#endif
                                }

                                std::cout << "**************************************************************" << std::endl;
                                std::cout << "a very good result with tri error " << tri_error << " will be taken " << std::endl;
                                std::cout << "**************************************************************" << std::endl;

                                /* break the current loop */
                                break;
                            }
                            else    // store not good result
                            {
                                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                                std::cout << "store rt for SfM with [wk, pre] ---> [" << id_wk << " , " << id_pre << " ]" << std::endl;
                                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                                SfM_Done = false;
                                image_rt.push_back(image_nxt);
                                pnt_cloud_rt.push_back(pnt_cloud_now);
                                camera_pose_rt.push_back(camera_pose_now);
                                tri_error_rt.push_back(tri_error);
                                id_rt.push_back(id_wk);
                            }
                        }
                    }
                }
            }
        }

        /* result judgment */
        if(!SfM_Done)
        {
            if(image_rt.empty())
            {
                if( step == 1 )
                {
                    step --;
                    _image_pre_list.pop_front();
                    _image_pre_list.push_front(image_wklist[0]);
#if FOR_IMG_AND_POSE
                    _amor_pose_pre = amor_pose_wklist[0];
#endif
                    for(int i = _DOSfMDISTANCE-1; i >= 1; i--)
                    {
                        _image_list.push_front(image_wklist[i]);
#if FOR_IMG_AND_POSE
                        _amor_pose_list.push_front(amor_pose_wklist[i]);
#endif
                    }
                    std::cout << "image_rt_list is empty, the image in pre_list is not suitable for the other, it will be threw out" << std::endl;
                    continue;
                }
                else
                {
                    std::cout << "image_rt_list is empty, error is happend in SfM PnP step, no results" << std::endl;
                    return false;
                }
            }

            /* get the result with minimon tri_error */
            float min_error = 1000; int right_i = 0;
            for(int i = 0; i < tri_error_rt.size(); i++)
            {
                if(min_error > tri_error_rt[i])
                {
                    min_error = tri_error_rt[i];
                    right_i = i;
                }
            }
            if(min_error < _error_of_bads)
            {
                /* get the suitable image , pose and pntcloud */
                image_nxt = image_rt[right_i];
                pnt_cloud_now = pnt_cloud_rt[right_i];
                camera_pose_now = camera_pose_rt[right_i];
                /* put the unused image , pose back to the list */
                for(int i = _DOSfMDISTANCE-1; i > id_rt[right_i]; i--)
                {
                    _image_list.push_front(image_wklist[i]);
#if FOR_IMG_AND_POSE
                    _amor_pose_list.push_front(amor_pose_wklist[i]);
#endif
                }
                std::cout << "**************************************************************" << std::endl;
                std::cout << "a ok result with tri error " << min_error << " will be taken " << std::endl;
                std::cout << "**************************************************************" << std::endl;
            }
            else    // TODO : add other limited conditions
            {
                if(step == 1)
                {
                    step --;
                    _image_pre_list.pop_front();
                    _image_pre_list.push_front(image_wklist[0]);
                    for(int i = _DOSfMDISTANCE-1; i >= 1; i--)
                    {
                        _image_list.push_front(image_wklist[i]);
                    }
                    std::cout << "**************************************************************" << std::endl;
                    std::cout << "the image in pre_list is not suitable for the other, it will be threw out" << std::endl;
                    std::cout << "**************************************************************" << std::endl;
                    continue;
                }
                else
                {
                    std::cout << " error is happend in SfM PnP step, all results are terriable , they are :" << std::endl;
                    std::cout << "-----------------------------------------------------------"<< std::endl;
                    std::cout << "| NUM | image | pnt_cloud(size) | tri_error |" << std::endl;
                    for(int i = 0; i < image_rt.size(); i++)
                    {
                        std::cout << "| " << i
                                  << " | " << id_rt[i]
                                  << " | " << pnt_cloud_rt[i].size()
                                  << " | " << tri_error_rt[i] << " |"
                                  << std::endl;
                    }

                    /* use sfm_pose and amor_pose to compute the new start point */


                    return false;
                }
            }
        }

        /* msg out */
        std::cout << "step " << step << " is completed ! " << std::endl;

        /* point cloud filter */
        std::vector<int> cloud_filter;
        {
            if(_USE_CLOUDFILTER)
            {
                cloud_filter = PointCloudFilter(pnt_cloud_now, _camera_pose_pre_list.front(), camera_pose_now);
            }
        }
        std::vector<int> bbox_filter;
        {
            if(_USE_BUNDINGBOX)
            {
                bbox_filter = BundingBoxFilter(pnt_cloud_now);
            }
        }


        /* store in _pnt_cloud_pre and _image_pre_list */
        {
            _pnt_cloud_pre_list.push_front(pnt_cloud_now);
            if(_pnt_cloud_pre_list.size() > _block_size) _pnt_cloud_pre_list.pop_back();

            _image_pre_list.push_front(image_nxt);
            if(_image_pre_list.size() > _block_size) _image_pre_list.pop_back();

            _camera_pose_pre_list.push_front(camera_pose_now);
            if(_camera_pose_pre_list.size() > _block_size) _camera_pose_pre_list.pop_back();
        }


        /* send out pnt cloud */
        {
            std::vector<int> status(pnt_cloud_now.size(), 1);
            std::vector<CloudPoint> pnt_cloud_send;
            if(_USE_CLOUDFILTER || _USE_BUNDINGBOX)
            {
                int cloud_count = 0, bbox_count = 0;
                for(int i = 0; i < status.size(); i++)
                {
                    if(_USE_CLOUDFILTER)
                    {
                        if(cloud_filter[i] == 0)
                        {
                            status[i] = 0;  cloud_count++;
                        }
                    }
                    if(_USE_BUNDINGBOX)
                    {
                        if(bbox_filter[i] == 0)
                        {
                            status[i] = 0;  bbox_count++;
                        }
                    }
                }
                for(int i = 0; i < status.size(); i++)
                {
                    if(status[i] == 1)
                        pnt_cloud_send.push_back(pnt_cloud_now[i]);
                }
                std::cout << "cloud filter and bundingbox filter : " << cloud_count << ", " << bbox_count << std::endl;
            }
            else
            {
                pnt_cloud_send = pnt_cloud_now;
            }
            /* send msg */
#if FOR_IMG_AND_POSE
            if(!SfMManagerCloudPoseSender(pnt_cloud_send, camera_pose_now, amor_pose_now)) // send msg
                std::cout << "send msg to ros in sfm_manager is error" << std::endl;
#else
            if(!SfMManagerCloudSender(pnt_cloud_send)) // send msg
                std::cout << "send msg to ros in sfm_manager is error" << std::endl;
#endif
        }
    }
}

/* ------------------------------------------------------------------------- */
/* \fn bool SfMManagerCloudPoseSender
*
* \brief send out the computet point cloud, camera pose and corresponding amor pose
*        they must have a same time stamp
*
* \param[in]  sfm_pntcloud --> target point cloud
*             camera_pose  --> target camera position
*             amor_pose    --> target amor position
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */

bool SFM_MANAGER::
SfMManagerCloudPoseSender(const std::vector<CloudPoint> sfm_pntcloud,
                      const cv::Matx34d camera_pose,
                      const cv::Matx34d amor_pose)
{
    if(sfm_pntcloud.empty())
    {
        std::cout << "input array pnt_cloud in function SendMsgToRos is empty" << std::endl;
        return false;
    }
    /* get msg */
    sensor_msgs::PointCloud pntcloud_msg;  geometry_msgs::PoseStamped camera_pose_msg;  geometry_msgs::PoseStamped amor_pose_msg;
    {
        ros::Time stamp = ros::Time::now();
        this->CVPointCloudToSensorPointCloudMsg(sfm_pntcloud, _send_id, stamp, pntcloud_msg);
        this->CVPoseToGeometryPoseMsg(camera_pose, _send_id, stamp, camera_pose_msg);
        this->CVPoseToGeometryPoseMsg(amor_pose, _send_id, stamp, amor_pose_msg);
    }
    /* send msg */
    _sfm_camera_pose_pub.publish(camera_pose_msg);
    _sfm_amor_pose_pub.publish(amor_pose_msg);
    _sfm_pntcloud_pub.publish(pntcloud_msg);
    _send_id++;

    return true;
}
/* ------------------------------------------------------------------------- */
/* \fn bool SfMManagerCloudSender
*
* \brief send out the computet point cloud
*
* \param[in]  sfm_pntcloud --> target point cloud
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool SFM_MANAGER::
SfMManagerCloudSender(const std::vector<CloudPoint>& sfm_pntcloud)
{
    if(sfm_pntcloud.empty())
    {
        std::cout << "input array pnt_cloud in function SendMsgToRos is empty" << std::endl;
        return false;
    }
    /* get msg */
    sensor_msgs::PointCloud pntcloud_msg;
    ros::Time stamp = ros::Time::now();
    this->CVPointCloudToSensorPointCloudMsg(sfm_pntcloud, _send_id, stamp, pntcloud_msg);
    _image_node_pub.publish(pntcloud_msg);
    _send_id++;

    return true;
}
/* ------------------------------------------------------------------------- */
/* \fn bool CVPointCloudToSensorPointCloudMsg
*
* \brief make point cloud to sensor_msgs::PointCloud
*
* \param[in]  sfm_pntcloud --> target point cloud
*             id --> send id for sensor_msgs::PointCloud.header.seq
*             stamp --> time stamp for sensor_msgs::PointCloud.header.stamp, for synchronization of messages
*
* \param[out] pnt_cloud_msg --> the complete msg
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool SFM_MANAGER::
CVPointCloudToSensorPointCloudMsg(const std::vector<CloudPoint> sfm_pnt_cloud,const int id, const ros::Time stamp,
                                  sensor_msgs::PointCloud& pnt_cloud_msg)
{
    if(sfm_pnt_cloud.empty())
    {
        std::cout << "input array pnt_cloud in function CvPointCloudToRosMsg is empty" << std::endl;
        return false;
    }
    /* create msg */
    {
        /* for header */
        {
            pnt_cloud_msg.header.seq = id;
            pnt_cloud_msg.header.stamp = stamp;
            pnt_cloud_msg.header.frame_id = "cloud";
        }
        /* points and channels */
        for(int i = 0; i < sfm_pnt_cloud.size(); i++)
        {
            CloudPoint sfm_pnt = sfm_pnt_cloud[i];
            geometry_msgs::Point32 pnt_info;
            pnt_info.x = sfm_pnt.object_pnt.x;
            pnt_info.y = sfm_pnt.object_pnt.y;
            pnt_info.z = sfm_pnt.object_pnt.z;
            pnt_cloud_msg.points.push_back(pnt_info);

            sensor_msgs::ChannelFloat32 extra_info;
            extra_info.values.push_back(sfm_pnt.pnt_color.val[0]);  // for colors
            extra_info.values.push_back(sfm_pnt.pnt_color.val[1]);
            extra_info.values.push_back(sfm_pnt.pnt_color.val[2]);

            extra_info.values.push_back(sfm_pnt.image_pnt_pre.x);   // for reference to img 1
            extra_info.values.push_back(sfm_pnt.image_pnt_pre.y);

            extra_info.values.push_back(sfm_pnt.image_pnt_nxt.x);   // for reference to img 2
            extra_info.values.push_back(sfm_pnt.image_pnt_nxt.y);
            pnt_cloud_msg.channels.push_back(extra_info);

        }
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool CVPoseToGeometryPoseMsg
*
* \brief make pose matrix to geometry_msgs::PoseStamped
*
* \param[in]  camera_pose --> target pose matrix
*             id --> send id for geometry_msgs::PoseStamped.header.seq
*             stamp --> time stamp for geometry_msgs::PoseStamped.header.stamp, for synchronization of messages
*
* \param[out] pose_msg --> the complete msg
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool SFM_MANAGER::
CVPoseToGeometryPoseMsg(const cv::Matx34d camera_pose, const int id, const ros::Time stamp,
                        geometry_msgs::PoseStamped &pose_msg)
{
    if(camera_pose == cv::Matx34d::zeros())
    {
        std::cout << "input matx camera_pose in function CVPoseToGeometryPoseMsg is empty" << std::endl;
        return false;
    }
    /* create msg */
    {
        /* for header */
        {
            pose_msg.header.seq = id;
            pose_msg.header.stamp = stamp;
            pose_msg.header.frame_id = id;
        }
        /* for position */
        {
            pose_msg.pose.position.x = camera_pose(0,3);
            pose_msg.pose.position.y = camera_pose(1,3);
            pose_msg.pose.position.z = camera_pose(2,3);
        }
        /* for orantation */
        {
            pose_msg.pose.orientation.w = sqrt(1.0+camera_pose(0,0)+camera_pose(1,1)+camera_pose(2,2))/2.0;
            float w4 = pose_msg.pose.orientation.w*4.0;
            pose_msg.pose.orientation.x = (camera_pose(2,1)-camera_pose(1,2))/w4;
            pose_msg.pose.orientation.y = (camera_pose(0,2)-camera_pose(2,0))/w4;
            pose_msg.pose.orientation.z = (camera_pose(1,0)-camera_pose(0,1))/w4;
        }
    }
    return true;
}
/* ------------------------------------------------------------------------- */
/* \fn bool Matx34ToRT
*
* \brief decomposition the 3*4 matrix to a 3*3 rotation matrix and a 3*1 translation matrix
*
* \param[in]  camera_pose --> target 3*4 matrix
*
* \param[out] rmatx --> 3*3 rotation matrix
*             tmatx --> 3*1 translation matrix
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool SFM_MANAGER::
Matx34ToRT(const cv::Matx34d camera_pose, cv::Matx33d &rmatx, cv::Matx31d& tmatx)
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
/* \fn std::vector<int> PointCloudFilter
*
* \brief in this function we make a filter for the point cloud.
*        the filter conditions are :
*        1). if the point is behind the cameras, it will be threw out
*        2). if the point is too far to the center or too near to the cameras, it will be threw out
*
* \param[in]  pnt_cloud --> target point cloud
*             camera_pose_pre --> camera position at time 1
*             camera_pose_nxt --> camera position at time 2
*
* \return   return a filter array, the element in the array shows the filter status,
*           if 1 , point will stay, if 0 , will be threw out
*/
/* ------------------------------------------------------------------------- */
std::vector<int> SFM_MANAGER::
PointCloudFilter(const std::vector<CloudPoint> pnt_cloud,
                 const cv::Matx34d camera_pose_pre,
                 const cv::Matx34d camera_pose_nxt)
{
    cv::Matx33d rmatx_nxt; cv::Matx31d tmatx_nxt; Matx34ToRT(camera_pose_nxt, rmatx_nxt, tmatx_nxt);
    cv::Matx33d rmatx_pre; cv::Matx31d tmatx_pre; Matx34ToRT(camera_pose_pre, rmatx_pre, tmatx_pre);

    cv::Point3d camera_position_pre = cv::Point3d(tmatx_pre(0), tmatx_pre(1), tmatx_pre(2));
    cv::Point3d camera_position_nxt = cv::Point3d(tmatx_nxt(0), tmatx_nxt(1), tmatx_nxt(2));

    /* when the point behind the camera, will be filten , and count the distance by the way */
    cv::Point3d pnts_center(0,0,0); int nonzero_num = 0; std::vector<int> status(pnt_cloud.size(), 1); // in status 1 means good, 0 means will be filter
    {
        for(int i = 0; i < pnt_cloud.size(); i++)
        {
            cv::Vec3d pnt_vec_nxt = pnt_cloud[i].object_pnt-camera_position_nxt;
            double cos_angle_nxt = pnt_vec_nxt.ddot(rmatx_nxt*cv::Vec3d(0,0,1))/cv::norm(pnt_vec_nxt);

            cv::Vec3d pnt_vec_pre = pnt_cloud[i].object_pnt-camera_position_pre;
            double cos_angle_pre = pnt_vec_pre.ddot(rmatx_pre*cv::Vec3d(0,0,1))/cv::norm(pnt_vec_pre);

            if(cos_angle_pre <= 0 || cos_angle_nxt <= 0)
            {
                status[i] = 0;
            }
            else
            {
                status[i] = 1;
                nonzero_num ++;
                pnts_center = pnts_center + pnt_cloud[i].object_pnt;
            }
        }
        pnts_center = cv::Point3d(pnts_center.x/double(nonzero_num), pnts_center.y/double(nonzero_num), pnts_center.z/double(nonzero_num));
    }

    /* compute distance to the pnts_center and camera position of every point */
    std::vector<double> dist_center(pnt_cloud.size()); double dist_center_average = 0.0;
    std::vector<double> dist_pose_pre(pnt_cloud.size()); double dist_pose_pre_average = 0.0;
    std::vector<double> dist_pose_nxt(pnt_cloud.size()); double dist_pose_nxt_average = 0.0;

    {
        for(int i = 0; i < pnt_cloud.size(); i++)
        {
            if(status[i] == 1)
            {
                dist_center[i] = cv::norm(pnt_cloud[i].object_pnt-pnts_center);
                dist_center_average += dist_center[i];

                dist_pose_pre[i] = cv::norm(pnt_cloud[i].object_pnt-camera_position_pre);
                dist_pose_pre_average += dist_pose_pre[i];

                dist_pose_nxt[i] = cv::norm(pnt_cloud[i].object_pnt-camera_position_nxt);
                dist_pose_nxt_average += dist_pose_nxt[i];
            }
        }
        dist_center_average = dist_center_average/double(nonzero_num);
        dist_pose_pre_average = dist_pose_pre_average/double(nonzero_num);
        dist_pose_nxt_average = dist_pose_nxt_average/double(nonzero_num);
    }

    /* filter the point, who has the too far distance to center and too near to camera */
    {
        for(int i = 0; i < dist_center.size(); i++)
        {
            if(status[i] == 1)
            {
                if(dist_center[i] > 2.0*dist_center_average ||
                   dist_pose_pre[i] < 0.05*dist_pose_pre_average ||
                   dist_pose_nxt[i] < 0.05*dist_pose_nxt_average )
                {
                    status[i] = 0;
                }
            }
        }
    }
    return status;
}

/* ------------------------------------------------------------------------- */
/* \fn void SyncCallback
*
* \brief Synchronized read the image and pose msg, and store image and pose in array
*
* \param[in]  pose_msg --> target pose message
*             image_msg --> target image message
*/
/* ------------------------------------------------------------------------- */
void SFM_MANAGER::
SyncCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg,
             const sensor_msgs::ImageConstPtr& image_msg)
{
    /* jump distance */
    _read_image_steps++;
    if(_read_image_steps%(int(_READSTEPDISTANCE*camera_fps)) != 0)    return;
    if(_read_image_steps > 100000)  _read_image_steps = 0;

    /* image */
    {
        cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGBA8);
        if(!cv_image_ptr->image.data)
        {   std::cout << "no new image data !" << std::endl;    return; }
        cv::Mat image = cv_image_ptr->image.clone();
        _image_list.push_back(image);
    }

    /* pose */
    {
        geometry_msgs::PoseStamped amor_pose;
        amor_pose.pose = pose_msg->pose;
        _amor_pose_list.push_back(PoseStampedToCVMatx(amor_pose));
    }
}

/* ------------------------------------------------------------------------- */
/* \fn cv::Matx34d  PoseStampedToCVMatx
*
* \brief make msg geometry_msgs::PoseStamped to 3*4 matrix
*
* \param[in]  pose_msg --> target message
*
* \return  return the corresponding 3*4 matrix
*/
/* ------------------------------------------------------------------------- */
cv::Matx34d SFM_MANAGER::
PoseStampedToCVMatx(const geometry_msgs::PoseStamped pose_msg)
{
    float qw = pose_msg.pose.orientation.w;
    float qx = pose_msg.pose.orientation.x;
    float qy = pose_msg.pose.orientation.y;
    float qz = pose_msg.pose.orientation.z;
    float t0 = pose_msg.pose.position.x;
    float t1 = pose_msg.pose.position.y;
    float t2 = pose_msg.pose.position.z;

    float sqw = qw*qw; float sqx = qx*qx; float sqy = qy*qy; float sqz = qz*qz;
    float invs = 1.0/(sqw + sqx + sqy + sqz);
    float m00 = ( sqx - sqy - sqz + sqw)*invs ;  // since sqw + sqx + sqy + sqz =1/invs*invs
    float m11 = (-sqx + sqy - sqz + sqw)*invs ;
    float m22 = (-sqx - sqy + sqz + sqw)*invs ;

    float xz = qx*qz;
    float yw = qy*qw;
    float xy = qx*qy;
    float zw = qz*qw;
    float yz = qy*qz;
    float xw = qx*qw;

    float m01 = 2.0 * (xy + zw)*invs;
    float m10 = 2.0 * (xy - zw)*invs;

    float m20 = 2.0 * (xz - yw)*invs ;
    float m02 = 2.0 * (xz + yw)*invs ;

    float m21 = 2.0 * (yz + xw)*invs ;
    float m12 = 2.0 * (yz - xw)*invs ;

    cv::Matx34d pose_temp(m00, m01, m02, t0,
                          m10, m11, m12, t1,
                          m20, m21, m22, t2);
    return pose_temp;
}


/* ------------------------------------------------------------------------- */
/* \fn std::vector<int> BundingBoxFilter
*
* \brief    make bunding box filter for point cloud.
*           filter conditions :
*           1). if there is a point too near to another, it will be threw out
*
* \param[in]  pnt_cloud --> target point cloud
*
* \return   return a filter array, the element in the array shows the filter status,
*           if 1 , point will stay, if 0 , will be threw out
*/
/* ------------------------------------------------------------------------- */
std::vector<int> SFM_MANAGER::
BundingBoxFilter(const std::vector<CloudPoint> pnt_cloud)
{
    std::vector<int> status(pnt_cloud.size(), 1);
    {
        for(int i = 0; i < pnt_cloud.size(); i++)
        {
            cv::Point3d pnt3d_i = pnt_cloud[i].object_pnt;
            if(status[i] == 1)
            {
                for(int ii = 1; ii < 0.4*pnt_cloud.size() && i+ii < pnt_cloud.size(); ii++)
                {
                    if(status[i+ii] == 1)
                    {
                        cv::Point3d pnt3d_ii = pnt_cloud[i+ii].object_pnt;
                        if(cv::norm(pnt3d_i-pnt3d_ii) < _bbox_radius)
                        {
                            status[i+ii] = 0;
                        }
                    }
                }
            }
        }
    }
    return status;
}
/* ------------------------------------------------------------------------- */
/* \fn void ImageNodeCallback
*
* \brief read the image form ros, and store it in array
*
* \param[in]      image_msg --> target image message
*/
/* ------------------------------------------------------------------------- */
void SFM_MANAGER::
ImageNodeCallback(const sensor_msgs::ImageConstPtr &image_msg)
{
    /* jump distance */
    _read_image_steps++;
    if(_read_image_steps%(int(_READSTEPDISTANCE*camera_fps)) != 0)    return;
    if(_read_image_steps > 100000)  _read_image_steps = 0;

    /* image */
    {
        cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGBA8);
        if(!cv_image_ptr->image.data)
        {   std::cout << "no new image data !" << std::endl;    return; }
        cv::Mat image = cv_image_ptr->image.clone();
        _image_list.push_back(image);
    }
}
