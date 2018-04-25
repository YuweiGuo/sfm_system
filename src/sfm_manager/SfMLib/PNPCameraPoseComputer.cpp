#include "PNPCameraPoseComputer.h"

/* ------------------------------------------------------------------------- */
/* \fn bool Init
*
* \brief    init parameter members in class
*
* \param[in]        keypnts_good_pre --> keypnts in image at time 1, they are resources for compute
*                   keypnts_good_nxt --> keypnts in image at time 2, they are resources for compute
*                   matches_good --> good matches, they are resources for compute
*                   cmaera_matx --> camera matrix
*                   distcoeff --> distcoeff
*                   pnt_cloud_preframe --> the point cloud, who war created by image 1 and image 1-n
*
* \return       if not parameters are false, renturn false. otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool PNPCameraPoseComputer::
Init(const std::vector<cv::KeyPoint> keypnts_good_pre,
     const std::vector<cv::KeyPoint> keypnts_good_nxt,
     const std::vector<cv::DMatch> matches_good,
     const cv::Matx33d camera_matx,
     const cv::Matx41d distcoeff,
     const std::vector<CloudPoint>& pnt_cloud_preframe)
{
    if(keypnts_good_pre.empty() || keypnts_good_nxt.empty() || matches_good.empty() )
    {
        SfM_ERROR("the init data for CameraPosComputer is empty, keypnts is empty");
        return false;
    }
    if(camera_matx == cv::Matx33d::zeros())
    {
        SfM_ERROR("the init data for CameraPosComputer is empty, or camera_matx is 0");
        return false;
    }
    if(pnt_cloud_preframe.empty())
    {
        SfM_ERROR("the init data for CameraPosComputer is empty, pnt_cloud_preframe is empty");
        return false;
    }
    if(keypnts_good_pre.size() != keypnts_good_nxt.size() || keypnts_good_pre.size() != matches_good.size())
    {
        SfM_ERROR("the init data for CameraPosComputer is failer, size of keypnts or/and matches are not same");
        return false;
    }
    _keypnts_good_pre.clear(); _keypnts_good_nxt.clear(); _matches_good.clear();
    for(int i = 0; i < keypnts_good_pre.size(); i++)
    {
        _keypnts_good_pre.push_back(keypnts_good_pre[i]);
        _keypnts_good_nxt.push_back(keypnts_good_nxt[i]);
        _matches_good.push_back(matches_good[i]);
    }
    _camera_matx = camera_matx;
    _distcoeff = distcoeff;
    _pnt_cloud_preframe.clear();
    for(int i = 0; i < pnt_cloud_preframe.size(); i++)
    {
        _pnt_cloud_preframe.push_back(pnt_cloud_preframe[i]);
    }
    _init_allready = true;
    SfM_SENDMSG("CameraPosComputer Init4PnP seccessful");
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool DoCameraPoseCompute
*
* \brief        the virtual function will be here implemented.
*               compute camera poses using cv::pnp
*               pipeline :
*               1). get the new keypoints and matches by computing fundamental matrix
*               2). run cv::pnp
*/
/* ------------------------------------------------------------------------- */
bool PNPCameraPoseComputer::
DoCameraPoseCompute()
{
    if(!_init_allready)
    {
        SfM_ERROR("CameraPosComputer has been not init for PnP methed if you want to use");
        return false;
    }
    SfM_SENDMSG("computing camera-pos with PnP ...");
    double time = cv::getTickCount();

    /* fundamental matrix */
    std::vector<cv::KeyPoint> keypnts_new_pre, keypnts_new_nxt;
    std::vector<cv::DMatch> matches_new;
    ComputeFundamentalMatx(_keypnts_good_pre, _keypnts_good_nxt, _matches_good, keypnts_new_pre, keypnts_new_nxt, matches_new);
    StoreKeyPntsMatchesNew(keypnts_new_pre, keypnts_new_nxt, matches_new);

    SfM_SENDMSG("keypnts_new have been found ...");

    /* camera pos pnp */
    cv::Matx34d camera_pose;
    if(!CameraPosPnPRansac(keypnts_new_pre, keypnts_new_nxt, _pnt_cloud_preframe, _camera_matx, _distcoeff, camera_pose))
    {
        SfM_ERROR("CameraPosPnPRansac faild in CameraPosComputerPnPMethod");
        return false;
    }

    StoreCameraPoseNxt(camera_pose);

    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    SfM_SENDMSG("Done. (" + std::to_string(time) +"s)");
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn unsigned int FindSame2D3DPoints
*
* \brief        in this function we will find the same 2D and 3D points,
*               who in the 2 image and 1 pointcloud
*
* \param[in]    keypnts_new_pre --> the keypnts in image at time 1
*               keypnts_new_nxt --> the keypnts in image at time 2
*               pnt_cloud_preframe --> the pntcloud, who was created by image 1 and image 1-n
*
* \param[out]   pnts2d_same_nxt --> the same 2D point in image at time 2
*               pnts3d_same_preframe --> the corresponding 3D points in pnt_cloud_preframe
*
* \return   return the number of the same points pairs
*/
/* ------------------------------------------------------------------------- */
unsigned int PNPCameraPoseComputer::
FindSame2D3DPoints(const std::vector<cv::KeyPoint> keypnts_new_pre,
                   const std::vector<cv::KeyPoint> keypnts_new_nxt,
                   const std::vector<CloudPoint> pnt_cloud_preframe,
                   std::vector<cv::Point2d>& pnts2d_same_nxt,
                   std::vector<cv::Point3d>& pnts3d_same_preframe)
{
    if(keypnts_new_pre.empty() || pnt_cloud_preframe.empty() || keypnts_new_pre.size() != keypnts_new_nxt.size())
    {
        SfM_ERROR("input parameter in FindSame2D3DPoints is/are empty or size of keypnts pre and nxt is not same");
        return -1;
    }

    std::vector<cv::Point2d> pnts_new_pre;  HelpFunc::KeyPntsToPnts(keypnts_new_pre, pnts_new_pre);
    std::vector<cv::Point2d> pnts_new_nxt;  HelpFunc::KeyPntsToPnts(keypnts_new_nxt, pnts_new_nxt);

    if(pnts_new_pre.empty() || pnts_new_nxt.empty())
    {
        SfM_ERROR("the pnts ,which are got from keypnts in FindSame2D3DPoints, is empty");
        return -1;
    }

    pnts2d_same_nxt.clear(); pnts3d_same_preframe.clear();
    std::set<int> existing_pnts3d;  // avoid duplication

    std::vector<int> status_pnts3d(pnt_cloud_preframe.size(), 0);
    int num_2d3d = 0;
    for(int i2d = 0; i2d < pnts_new_pre.size(); i2d++)
    {
        for(int i3d = 0; i3d < pnt_cloud_preframe.size(); i3d++)
        {
            if(status_pnts3d[i3d] == 0 && pnts_new_pre[i2d] == pnt_cloud_preframe[i3d].image_pnt_nxt)
            {
                num_2d3d++;
                status_pnts3d[i3d] == 1;
                pnts2d_same_nxt.push_back(pnts_new_nxt[i2d]);
                pnts3d_same_preframe.push_back(pnt_cloud_preframe[i3d].object_pnt);
            }
        }
    }
    SfM_SENDMSG("same 2D3D points have been found, same 2D3D points : "+std::to_string(num_2d3d));
    return num_2d3d;
}

/* ------------------------------------------------------------------------- */
/* \fn bool CameraPosPnPRansac
*
* \brief        use cv::pnp method to compute the camera pose.
*               there is the error control too.
*
* \param[in]    keypnts_new_pre --> the keypnts in image at time 1
*               keypnts_new_nxt --> the keypnts in image at time 2
*               pnt_cloud_preframe --> the pntcloud, who was created by image 1 and image 1-n
*               camera_matx --> camera matrix
*               distcoeff --> distcoeff
*
* \param[out]   camera_pose --> calculated camera pose for image at time 2
*
* \return       if seccessful return true , otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool PNPCameraPoseComputer::
CameraPosPnPRansac(const std::vector<cv::KeyPoint> keypnts_new_pre,
                   const std::vector<cv::KeyPoint> keypnts_new_nxt,
                   const std::vector<CloudPoint> pnt_cloud_preframe,
                   const cv::Matx33d camera_matx,
                   const cv::Matx41d distcoeff,
                   cv::Matx34d& camera_pose)
{
    static int firstexc = 1;
    static int count = 2016;
    count ++;
    if(firstexc)
    {
        std::srand(time(NULL));
        firstexc = 0;
    }
    cv::theRNG().state = (std::rand()%10000) + count;

    /* find same pnts 2d3d from keypnts_new_nxt and pnt_cloud_preframe */
    std::vector<cv::Point2d> pnts2d_same_nxt; std::vector<cv::Point3d> pnts3d_same_preframe;
    unsigned int num_same_pnts =
            FindSame2D3DPoints(keypnts_new_pre, keypnts_new_nxt, pnt_cloud_preframe, pnts2d_same_nxt, pnts3d_same_preframe);
    if(num_same_pnts < 7)
    {
        SfM_ERROR("there are not enough same 2d3d points for PnP method, same points : "+std::to_string(num_same_pnts));
        return false;
    }

    if(pnts2d_same_nxt.size() != pnts3d_same_preframe.size() || pnts2d_same_nxt.empty())
    {
        SfM_ERROR("there are not same num in 2d3d points vectors for PnP method or pnts2d3d are empty");
        return false;
    }
    /* pnp method */
    std::vector<int> inliers;
    double min_val, max_val; cv::minMaxIdx(pnts2d_same_nxt, &min_val, &max_val);
    cv::Mat_<double> rvec, tvec;
    cv::solvePnPRansac(pnts3d_same_preframe, pnts2d_same_nxt, cv::Mat_<double>(camera_matx), cv::Mat_<double>(distcoeff),
                       rvec, tvec, true, 1000, 0.006*max_val, 0.25*(double)(pnts2d_same_nxt.size()), inliers, CV_EPNP);

    /* check RT matrix with reprojection points */
    std::vector<cv::Point3d> pnts3d_preframe; HelpFunc::Get3DPointsFromPointCloud(pnt_cloud_preframe, pnts3d_preframe);
    std::vector<cv::Point2d> pnts2d_new_nxt; HelpFunc::KeyPntsToPnts(keypnts_new_nxt, pnts2d_new_nxt);
    std::vector<cv::Point2d> pnts_reprojected2d;

    cv::projectPoints(pnts3d_preframe, rvec, tvec, camera_matx, distcoeff, pnts_reprojected2d);

    if(inliers.size() == 0) //get inliers
    {
        for(int i = 0; i < pnts_reprojected2d.size(); i++)
        {
            if(cv::norm(pnts_reprojected2d[i]-pnts2d_new_nxt[i]) < 10.0)
                inliers.push_back(i);
        }
    }

    if(inliers.size() < (double)(pnts2d_new_nxt.size())/5.0)
    {
        SfM_ERROR("not enough inliers to consider a good pose : "+std::to_string(float(inliers.size())/float(pnts2d_new_nxt.size())));
        return false;
    }else
        SfM_SENDMSG("enough inliers to consider a good pose : "+std::to_string(float(inliers.size())/float(pnts2d_new_nxt.size())));


    /* check R and T */
    if(cv::norm(tvec) > 200.0)
    {
        SfM_ERROR("estimated camera movement is too big in PnP method");
        return false;
    }
    cv::Mat_<double> R;
    cv::Rodrigues(rvec, R);
    camera_pose = cv::Matx34d(R(0,0), R(0,1), R(0,2), tvec(0),
                             R(1,0), R(1,1), R(1,2), tvec(1),
                             R(2,0), R(2,1), R(2,2), tvec(2));

    return true;
}
