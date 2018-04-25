#include "SfMTool.h"


/* ------------------------------------------------------------------------- */
/* \fn SfMTool
*
* \brief    constructor for SfMTool
*
* \param[in]    opencv_nonfree_init --> if SfMTool want to be created, the cv::initModule_nonfree() must be given as this parameter.
*
*/
/* ------------------------------------------------------------------------- */
SfMTool::SfMTool(bool opencv_nonfree_init)
{
    if(opencv_nonfree_init == true)
    {
        CLASS_CREATE_MSG("SfMTool", true);
    }
    else
    {
        SfM_ERROR("you must use cv::initModule_nonfree() as the input parameter for create SfMTool");
        CLASS_CREATE_MSG("SfMTool", false);
    }
}

/* ------------------------------------------------------------------------- */
/* \fn SfMTool
*
* \brief    constructor for SfMTool
*
* \param[in]    FeatureDetecteMethod --> for create the SfM component
*               DescriptorExtractorMethod --> for create the SfM component
*               opencv_nonfree_init --> if SfMTool want to be created, the cv::initModule_nonfree() must be given as this parameter.
*/
/* ------------------------------------------------------------------------- */
SfMTool::SfMTool(std::string FeatureDetecteMethod,
                 std::string DescriptorExtractorMethod,
                 bool opencv_nonfree_init) : _feature_matcher(FeatureDetecteMethod, DescriptorExtractorMethod)
{
    if(opencv_nonfree_init == true)
    {
        CLASS_CREATE_MSG("SfMTool with self defined feature method", true);
    }
    else
    {
        SfM_ERROR("you must use cv::initModule_nonfree() as the input parameter for create SfMTool");
        CLASS_CREATE_MSG("SfMTool", false);
    }
}

/* ------------------------------------------------------------------------- */
/* \fn bool DoSfM42ImagesWithEPG
*
* \brief    do SfM for 2 images with epg method
*
* \param[in]    image_pre --> image at time 1
*               image_nxt --> image at time 2
*               camera_pose_pre --> camera position at time 1
*               epg_firstlength --> the move unit of this epg method, it has a default value 1
*
* \param[out]   pnt_cloud_now --> calculated point cloud from this method
*               camera_pose_now --> calculated position of camera at time 2
*               error --> reprojection error of the points in pnt_cloud_now
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool SfMTool::
DoSfM42ImagesWithEPG(const cv::Mat image_pre,
                     const cv::Mat image_nxt,
                     const cv::Matx34d& camera_pose_pre,
                     std::vector<CloudPoint> &pnt_cloud_now,
                     cv::Matx34d &camera_pose_now,
                     float& error,
                     const float epg_firstlength)
{
    /* check input parameters */
    if(image_pre.empty() || image_nxt.empty())
    {
        SfM_ERROR("image_pre/_nxt is/are empty");
        return false;
    }

    double time = cv::getTickCount();

    SfM_SENDMSG("doing sfm for 2 images with epg method");
    /* feature matching */
    std::vector<cv::KeyPoint> keypnts_good_pre, keypnts_good_nxt;
    std::vector<cv::DMatch> matches_good;

    {
        if(!_feature_matcher.Init(image_pre, image_nxt)) return false;
        if(!_feature_matcher.DoFeatureMatching())   return false;
        if(!_feature_matcher.GetGoodKeyPnts(keypnts_good_pre, keypnts_good_nxt)) return false;
        if(!_feature_matcher.GetGoodMachtes(matches_good)) return false;
    }

    /* compute camera pos */
    cv::Matx34d camera_pose_nxt;
    std::vector<cv::KeyPoint> keypnts_new_pre, keypnts_new_nxt;

    {
        if(!_epg_camera_pose_computer.Init(keypnts_good_pre, keypnts_good_nxt, matches_good, _camera_matx, _distcoeff, epg_firstlength)) return false;
        if(!_epg_camera_pose_computer.DoCameraPoseCompute()) return false;

        if(!_epg_camera_pose_computer.GetKeyPntsNew(keypnts_new_pre, keypnts_new_nxt)) return false;
        if(!_epg_camera_pose_computer.GetCameraPose(camera_pose_nxt)) return false;
    }

    /* triangulation */
    std::vector<CloudPoint> pnt_cloud;

    {
        if(!_triangulator.Init(image_pre, image_nxt, keypnts_new_pre, keypnts_new_nxt, _camera_matx, camera_pose_pre, camera_pose_nxt)) return false;
        error = _triangulator.DoTriangulation();

        SfM_SENDMSG("triangulation error is "+std::to_string(error));
        if(!_triangulator.GetPointCloud(pnt_cloud)) return false;
    }

    /* store result */
    camera_pose_now = camera_pose_nxt;
    for(int i = 0; i < pnt_cloud.size(); i++)
    {
        pnt_cloud_now.push_back(pnt_cloud[i]);
    }

    /* return */
    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    SfM_SENDMSG("sfm for 2 images is done!(" + std::to_string(time) +"s)");
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool DoSfM4ImagesWithPNP
*
* \brief    do SfM for 2 images with pnp method
*
* \param[in]    image_pre --> image at time 1
*               image_nxt --> image at time 2
*               camera_pose_pre --> camera position at time 1
*               pnt_cloud_pre --> the point cloud, which war created by image at time 1 and image at time 1-n
*
* \param[out]   pnt_cloud_now --> calculated point cloud from this method
*               camera_pose_now --> calculated position of camera at time 2
*               error --> reprojection error of the points in pnt_cloud_now
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool SfMTool::
DoSfM4ImagesWithPNP(const cv::Mat image_pre,
                    const cv::Mat image_nxt,
                    const cv::Matx34d& camera_pose_pre,
                    const std::vector<CloudPoint>& pnt_cloud_pre,
                    std::vector<CloudPoint>& pnt_cloud_now,
                    cv::Matx34d& camera_pose_now,
                    float& error)
{
    /* check parameters */
    if(image_pre.empty() || image_nxt.empty())
    {
        SfM_ERROR("image_pre/_nxt is/are empty");
        return false;
    }

    double time = cv::getTickCount();
    SfM_SENDMSG("doing sfm for 2 images with pnp method");
    /* feature matching */
    std::vector<cv::KeyPoint> keypnts_good_pre, keypnts_good_nxt;
    std::vector<cv::DMatch> matches_good;

    {
        if(!_feature_matcher.Init(image_pre, image_nxt)) return false;
        if(!_feature_matcher.DoFeatureMatching())   return false;
        if(!_feature_matcher.GetGoodKeyPnts(keypnts_good_pre, keypnts_good_nxt)) return false;
        if(!_feature_matcher.GetGoodMachtes(matches_good)) return false;
    }

    /* compute camera pos */
    cv::Matx34d camera_pose_nxt;
    std::vector<cv::KeyPoint> keypnts_new_pre, keypnts_new_nxt;

    {
        if(!_pnp_camera_pose_computer.Init(keypnts_good_pre, keypnts_good_nxt, matches_good, _camera_matx, _distcoeff, pnt_cloud_pre)) return false;
        if(!_pnp_camera_pose_computer.DoCameraPoseCompute()) return false;

        if(!_pnp_camera_pose_computer.GetKeyPntsNew(keypnts_new_pre, keypnts_new_nxt)) return false;
        if(!_pnp_camera_pose_computer.GetCameraPose(camera_pose_nxt)) return false;
    }

    /* triangulation */
    std::vector<CloudPoint> pnt_cloud;

    {
        if(!_triangulator.Init(image_pre, image_nxt, keypnts_new_pre, keypnts_new_nxt, _camera_matx, camera_pose_pre, camera_pose_nxt)) return false;
        error = _triangulator.DoTriangulation();

        SfM_SENDMSG("triangulation error is "+std::to_string(error));
        if(!_triangulator.GetPointCloud(pnt_cloud)) return false;
    }

    /* store result */
    camera_pose_now = camera_pose_nxt;
    for(int i = 0; i < pnt_cloud.size(); i++)
    {
        pnt_cloud_now.push_back(pnt_cloud[i]);
    }

    /* return */
    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    SfM_SENDMSG("sfm for 2 images is done!(" + std::to_string(time) +"s)");
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool Init
*
* \brief    init the SfMTool object
*
* \param[in]    camera_matx --> camera matrix
*               distcoeff --> dist coeffe
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool SfMTool::
Init(const cv::Matx33d camera_matx, const cv::Matx41d distcoeff)
{
    if(camera_matx == cv::Matx33d::zeros())
    {
        SfM_ERROR("camera_matx == 0 in SfMTool init");
        return false;
    }
    _camera_matx = camera_matx;
    _distcoeff = distcoeff;
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool DoSfM42ImagesWithPoses
*
* \brief    do SfM for 2 images with positions, the positions can be got from other resources
*
* \param[in]    image_pre --> image at time 1
*               image_nxt --> image at time 2
*               camera_pose_pre --> camera position at time 1
*               camera_pose_nxt --> camera position at time 2
*
* \param[out]   pnt_cloud_now --> calculated point cloud from this method
*               error --> reprojection error of the points in pnt_cloud_now
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool SfMTool::
DoSfM42ImagesWithPoses(const cv::Mat image_pre, const cv::Matx34d camera_pose_pre,
                       const cv::Mat image_nxt, const cv::Matx34d camera_pose_nxt,
                       std::vector<CloudPoint>& pnt_cloud_now,
                       float& error)
{
    /* check parameters */
    if(image_pre.empty() || image_nxt.empty())
    {
        SfM_ERROR("image_pre/_nxt is/are empty");
        return false;
    }

    double time = cv::getTickCount();

    SfM_SENDMSG("doing sfm for 2 images");

    /* feature matching */
    std::vector<cv::KeyPoint> keypnts_good_pre, keypnts_good_nxt;
    std::vector<cv::DMatch> matches_good;

    {
        if(!_feature_matcher.Init(image_pre, image_nxt)) return false;
        if(!_feature_matcher.DoFeatureMatching())   return false;
        if(!_feature_matcher.GetGoodKeyPnts(keypnts_good_pre, keypnts_good_nxt)) return false;
        if(!_feature_matcher.GetGoodMachtes(matches_good)) return false;
    }

    /* find new matches */
    std::vector<cv::KeyPoint> keypnts_new_pre, keypnts_new_nxt;
    std::vector<cv::DMatch> matches_new;
    {
        _epg_camera_pose_computer.ComputeFundamentalMatx(keypnts_good_pre, keypnts_good_nxt, matches_good,
                                                         keypnts_new_pre, keypnts_new_nxt, matches_new);
    }

    /* triangulation */
    std::vector<CloudPoint> pnt_cloud;

    {
        if(!_triangulator.Init(image_pre, image_nxt, keypnts_new_pre, keypnts_new_nxt, _camera_matx, camera_pose_pre, camera_pose_nxt)) return false;
        error = _triangulator.DoTriangulation();

        SfM_SENDMSG("triangulation error is "+std::to_string(error));
        if(!_triangulator.GetPointCloud(pnt_cloud)) return false;
    }

    /* store result */
    for(int i = 0; i < pnt_cloud.size(); i++)
    {
        pnt_cloud_now.push_back(pnt_cloud[i]);
    }

    /* return */
    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    SfM_SENDMSG("sfm for 2 images with corresponding poses is done!(" + std::to_string(time) +"s)");
    return true;
}
