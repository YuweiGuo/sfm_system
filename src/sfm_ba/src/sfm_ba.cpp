#include "sfm_ba.h"

/* ------------------------------------------------------------------------- */
/* \fn void SfMBACloudSender
*
* \brief    this function send out the completed point cloud.
*           the clouds are in the _pnt_cloud_wk_list
*/
/* ------------------------------------------------------------------------- */
void SFM_BA::
SfMBACloudSender()
{
    sensor_msgs::PointCloud cloud_msg;
    GetPointCloudMsg(cloud_msg);
    this->_sfm_ba_cloud_pub.publish(cloud_msg);
}

/* ------------------------------------------------------------------------- */
/* \fn void GetPointCloudMsg
*
* \brief    get sensor_msg::PointCloud message from _pnt_cloud_wk_list.
*
* \param[out]    cloud_msg --> target point cloud message
*/
/* ------------------------------------------------------------------------- */
void SFM_BA::
GetPointCloudMsg(sensor_msgs::PointCloud& cloud_msg)
{
    for(int cloud_num = 0; cloud_num < _pnt_clouds_wk.size(); cloud_num++)
    {
        for(int i = 0; i < _pnt_clouds_wk[cloud_num].size(); i++)
        {
            CloudPoint ba_pnt = _pnt_clouds_wk[cloud_num][i];
            geometry_msgs::Point32 pnt_info;
            pnt_info.x = ba_pnt.object_pnt.x;
            pnt_info.y = ba_pnt.object_pnt.y;
            pnt_info.z = ba_pnt.object_pnt.z;
            cloud_msg.points.push_back(pnt_info);

            sensor_msgs::ChannelFloat32 extra_info;
            extra_info.values.push_back(ba_pnt.pnt_color.val[0]);  // for colors
            extra_info.values.push_back(ba_pnt.pnt_color.val[1]);
            extra_info.values.push_back(ba_pnt.pnt_color.val[2]);

            extra_info.values.push_back(ba_pnt.image_pnt_pre.x);   // for reference to img 1
            extra_info.values.push_back(ba_pnt.image_pnt_pre.y);

            extra_info.values.push_back(ba_pnt.image_pnt_nxt.x);   // for reference to img 2
            extra_info.values.push_back(ba_pnt.image_pnt_nxt.y);
            cloud_msg.channels.push_back(extra_info);

        }
    }
}

/* ------------------------------------------------------------------------- */
/* \fn void Run
*
* \brief    for outside call in the main function, start sfm_ba process
*
*/
/* ------------------------------------------------------------------------- */
void SFM_BA::
Run()
{
    Start();
    Stop();
}

/* ------------------------------------------------------------------------- */
/* \fn void Start
*
* \brief    start Ros loop, and sfm-ba process
*/
/* ------------------------------------------------------------------------- */
void SFM_BA::
Start()
{
    std::cout << "BA node is startet!" << std::endl;
    _spinner.start();
    MsgLoop();
}

/* ------------------------------------------------------------------------- */
/* \fn void Stop
*
* \brief    stop Ros loop
*/
/* ------------------------------------------------------------------------- */
void SFM_BA::
Stop()
{
    _spinner.stop();
}

/* ------------------------------------------------------------------------- */
/* \fn void MsgLoop
*
* \brief    do ba in the ros loop
*/
/* ------------------------------------------------------------------------- */
void SFM_BA::
MsgLoop()
{
    std::cout << "waiting for BA request..." << std::endl;
    while(ros::ok())
    {
        DoSfMBA();
    }
    std::cout << "BA node is end." << std::endl;
}

/* ------------------------------------------------------------------------- */
/* \fn bool DoSfMBA
*
* \brief    the function read clouds and poses from receive_list,
*           and put the resources to the worklist.
*           then do sfm_ba wiht SfMLib for more than 10 clouds and every 5 more clouds.
*           an the end send out the result
*/
/* ------------------------------------------------------------------------- */
bool SFM_BA::
DoSfMBA()
{
    /* new received cloud < 3 return */
    if(_pnt_clouds_rv.size() <= 3)
        return false;
    /* get point cloud , camera pose from _pnt_cloud_rv , camera pose rv*/
    {
        _lock_rv.lock();
        int rv_size = _pnt_clouds_rv.size();
        while(rv_size >= 0)
        {
            _pnt_clouds_wk.push_back(_pnt_clouds_rv.front());
            _pnt_clouds_rv.pop_front();
            _camera_poses_wk.push_back(_camera_poses_rv.front());
            _camera_poses_rv.pop_front();
            rv_size--;
        }
        _lock_rv.unlock();
    }
    if(_camera_poses_wk.size()%5 != 0 && _camera_poses_wk.size() >= 10)
        return false;
    /* do ba */
    {
        _sfm_tool._bundleadjustor.Init(camera_matx, dist_coeff, _pnt_clouds_wk, _camera_poses_wk);
        if(!_sfm_tool._bundleadjustor.DoBundleAdjust())
        {
            std::cout << "error in sfm_ba node func : DoSfMBA()" << std::endl;
            return false;
        }
    }
    SfMBACloudSender();
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn void SyncCallback
*
* \brief    synchronized receive clouds msg and poses msg, and store in corresponding receive_list
*
* \param[in]    pose_msg --> the target posistion message
*               cloud_msg --> the target clouds message
*/
/* ------------------------------------------------------------------------- */
void SFM_BA::
SyncCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::PointCloudConstPtr& cloud_msg)
{
    cv::Matx34d camera_pose;    PoseStamped2Matx(pose_msg, camera_pose);


    std::vector<CloudPoint> pnt_cloud;
    for(int i = 0; i < cloud_msg->points.size(); i++)
    {
        CloudPoint pnt;
        pnt.object_pnt.x = cloud_msg->points[i].x;
        pnt.object_pnt.y = cloud_msg->points[i].y;
        pnt.object_pnt.z = cloud_msg->points[i].z;

        pnt.pnt_color.val[0] = cloud_msg->channels[i].values[0];
        pnt.pnt_color.val[1] = cloud_msg->channels[i].values[1];
        pnt.pnt_color.val[2] = cloud_msg->channels[i].values[2];

        pnt.image_pnt_pre.x = cloud_msg->channels[i].values[3];
        pnt.image_pnt_pre.y = cloud_msg->channels[i].values[4];

        pnt.image_pnt_nxt.x = cloud_msg->channels[i].values[5];
        pnt.image_pnt_nxt.y = cloud_msg->channels[i].values[6];
        pnt_cloud.push_back(pnt);
    }
    _lock_rv.lock();
    _camera_poses_rv.push_back(camera_pose);
    _pnt_clouds_rv.push_back(pnt_cloud);
    _lock_rv.unlock();

}

/* ------------------------------------------------------------------------- */
/* \fn void PoseStamped2Matx
*
* \brief    make pose_msg to 3*4 matrix
*
* \param[in]    pose_msg --> traget pose message
*
* \Param[out]   pose_matx --> target 3*4 message
*/
/* ------------------------------------------------------------------------- */
void SFM_BA::
PoseStamped2Matx(const geometry_msgs::PoseStampedConstPtr &pose_msg, cv::Matx34d pose_matx)
{
    /* for die amor pose */
    float qw = pose_msg->pose.orientation.w;
    float qx = pose_msg->pose.orientation.x;
    float qy = pose_msg->pose.orientation.y;
    float qz = pose_msg->pose.orientation.z;
    float t0 = pose_msg->pose.position.x;
    float t1 = pose_msg->pose.position.y;
    float t2 = pose_msg->pose.position.z;

    float sqw = qw*qw; float sqx = qx*qx; float sqy = qy*qy; float sqz = qz*qz;
    float m00 = 1.0 - 2.0*(sqy + sqz) ;  // since sqw + sqx + sqy + sqz =1/invs*invs
    float m11 = 1.0 - 2.0*(sqx + sqz) ;
    float m22 = 1.0 - 2.0*(sqx + sqy) ;

    float xz = qx*qz;
    float yw = qy*qw;
    float xy = qx*qy;
    float zw = qz*qw;
    float yz = qy*qz;
    float xw = qx*qw;

    float m10 = 2.0*(xy + zw);
    float m01 = 2.0*(xy - zw);

    float m20 = 2.0*(xz - yw);
    float m02 = 2.0*(xz + yw);

    float m21 = 2.0*(yz + xw);
    float m12 = 2.0*(yz - xw);

    cv::Matx34d matx(m00, m01, m02, t0,
                     m10, m11, m12, t1,
                     m20, m21, m22, t2);
    pose_matx = matx;
}

