#include "CVBundleAdjustor.h"

/* ------------------------------------------------------------------------- */
/* \fn  bool DoBundleAdjust
*
* \brief    do bundle adjustment for the parameter members
*/
/* ------------------------------------------------------------------------- */
bool CVBundleAdjustor::
DoBundleAdjust()
{
    if(!_init_allready)
    {
        SfM_ERROR("BundleAdjustor should be inited first");
        return false;
    }
    if(_pnt_clouds.empty())
    {
        SfM_ERROR("input parameter pnt_cloud in BundleAdjustor::DoCVBundleAdjust42 is empty");
        return false;
    }
    if(_camera_poses.size() != _pnt_clouds.size()+1)
    {
        SfM_ERROR("size of input parameter _camera_poses in BundleAdjustor::DoCVBundleAdjust42 is not pnt_cloud.size()+1");
        return false;
    }
    double time = cv::getTickCount();
    SfM_SENDMSG("doing BA...");
    /* nums */
    int64 num = 0; // num of object pnts
    for(int i = 0; i < _pnt_clouds.size(); i++)
    {
        num += (_pnt_clouds[i]).size();
    }
    const int64 pnts_num = num; // num of pnts3d
    const unsigned int views_num = _camera_poses.size();   // num of views
#if DEBUG
    SfM_SENDMSG("in BA _pnt_clouds_num , views_num and pnts_num have been got : "+
                   std::to_string(_pnt_clouds.size())+", "+
                   std::to_string(views_num)+", "+std::to_string(pnts_num));
#endif

    /* init camera matx vec */
    std::vector<cv::Mat> camera_matx_vec(CV_32FC1);    // camera matrix
    {
        camera_matx_vec.clear();
        for(int i = 0; i < views_num; i++)
        {
            camera_matx_vec.push_back(cv::Mat(_camera_matx));
        }
    }
#if DEBUG
    SfM_SENDMSG("in BA camera_matx_vec init complete");
#endif

    /* init camera rt matx */
    std::vector<cv::Mat> rmatx_vec(CV_32FC1);          // rotation matrices of cameras (input and output)
    std::vector<cv::Mat> tmatx_vec(CV_32FC1);      	 // translation vector of cameras (input and output)
    {
        rmatx_vec.clear(); tmatx_vec.clear();
        for(int i = 0; i < views_num; i++)
        {
            cv::Matx33d rmatx;  cv::Matx31d tmatx;
            if(!HelpFunc::Matx34ToRT(_camera_poses[i], rmatx, tmatx))
            {
                return false;
            }
            rmatx_vec.push_back(cv::Mat(rmatx)); tmatx_vec.push_back(cv::Mat(tmatx));
        }
    }
#if DEBUG
    SfM_SENDMSG("in BA rt_vec init complete");
#endif

    /* init distcoeff */
//    cv::Mat_<double> distcoeff(5,1,CV_32FC1);
//    distcoeff.at<double>(0,0) = _distcoeff(0);
//    distcoeff.at<double>(1,0) = _distcoeff(1);
//    distcoeff.at<double>(2,0) = _distcoeff(2);
//    distcoeff.at<double>(3,0) = _distcoeff(3);
//    distcoeff.at<double>(4,0) = 0;
    std::vector<cv::Mat> distcoeff_vec(CV_32FC1);		// distortion coefficients of 2 cameras (input and output)
    {
        distcoeff_vec.clear();
        for(int i = 0; i < views_num; i++)
        {
            distcoeff_vec.push_back(cv::Mat(_distcoeff));
        }
    }
#if DEBUG
    SfM_SENDMSG("in BA distcoeff_vec have been got");
#endif


    /* need variable pnts3d cooresponding images and pnts visibilities */
    std::vector<cv::Point3d> object_pnts;   // point3d from pnt_cloud (input and output)
    std::vector<std::vector<cv::Point2d> > image_pnts;  // cooresponding image pnts in 2 images for object_pnts
    std::vector<std::vector<int> > objpnts_vs;  // object-pnts-visibility for 2 cameras, as default can not be seen

    /* get 3d point, coorisponding image pnts and object pnts visibalities from pnt_cloud */
    {
        if(!GetImagepntsVSandPnt3D(pnts_num, views_num, _pnt_clouds, image_pnts, objpnts_vs, object_pnts, _range))
        {
            SfM_ERROR("function GetImagepntsVSandPnt3D in BundleAdjustor::DoCVBundleAdjust42 went error");
            return false;
        }
    }
#if DEBUG
    SfM_SENDMSG("in BA image-pnts object-pnts-vs and pnts3d have been got");
#endif


#if DEBUG
    SfM_SENDMSG("check parameters before do cvBA");
    SfM_SENDMSG("size of object_pnts : "+std::to_string(object_pnts.size()));
    SfM_SENDMSG("for image_pnts : ");
    for(int i = 0; i < image_pnts.size(); i++)
    {
        SfM_SENDMSG("view "+std::to_string(i)+" : size of image_pnts is "+std::to_string(image_pnts[i].size()));
    }
    SfM_SENDMSG("for objpnts_vs : ");
    for(int i = 0; i < objpnts_vs.size(); i++)
    {
        SfM_SENDMSG("view "+std::to_string(i)+" : size of image_pnts is "+std::to_string(objpnts_vs[i].size()));
    }
    SfM_SENDMSG("size of camera_matx_vec : "+std::to_string(camera_matx_vec.size()));
    SfM_SENDMSG("size of rmatx_vec : "+std::to_string(rmatx_vec.size()));
    SfM_SENDMSG("size of tmatx_vec : "+std::to_string(tmatx_vec.size()));
    SfM_SENDMSG("size of distcoeff_vec : "+std::to_string(distcoeff_vec.size()));
#endif


    /* do BA */
    SfM_SENDMSG("doing cvBA...");
    cv::LevMarqSparse::bundleAdjust(object_pnts, image_pnts, objpnts_vs, camera_matx_vec,
                                    rmatx_vec, tmatx_vec, distcoeff_vec);
    SfM_SENDMSG("cvBA is done");

#if DEBUG
    SfM_SENDMSG("in BA doing cv::LevMarqSparse::bundleAdjust seccesful");
#endif

    /* get back the new pnts and poses */

    if(object_pnts.size() != pnts_num)
    {
        SfM_ERROR("in Ba object_pnts.size() has been changed");
        return false;
    }
    for(int i = 0; i < _pnt_clouds.size(); i++)
    {
        for(int j = 0; i < _pnt_clouds.size(); i++)
        {
            (_pnt_clouds[i])[i].object_pnt = object_pnts[i];
        }
    }

#if DEBUG
    SfM_SENDMSG("in BA new _pnt_clouds has been stored");
#endif

    if(rmatx_vec.size() != views_num || tmatx_vec.size() != views_num)
    {
        SfM_ERROR("through bundleAdjust, size of rmatx_vec or/and tmatx is not 2 in BA4N");
        return false;
    }
    _camera_poses.clear();
    for(int i = 0; i < views_num; i++)
    {
        cv::Matx34d camera_pose_new;
        HelpFunc::RTToMatx34(cv::Matx33d(rmatx_vec[i]), cv::Matx31d(tmatx_vec[i]), camera_pose_new);
        _camera_poses.push_back(camera_pose_new);
    }

#if DEBUG
    SfM_SENDMSG("in BA new _camera_poses has been stored");
#endif

    /* complete */
    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    SfM_SENDMSG("BA4N Done. (" + std::to_string(time) +"s)");

    _ba_allready = true;
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn  bool Init
*
* \brief    init the parameter members in class
*
* \param[in]    camera_matx --> camera matrix
*               distcoeff --> dist coeff
*               pnt_clouds --> the point clouds, who want to make ba
*               camera_poses --> the camera poses, who want to make ba
*               range --> the search range in GetImagepntsVSandPnt3D.
*
* \return   if parameters have error return false, otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool CVBundleAdjustor::
Init(const cv::Matx33d camera_matx, const cv::Matx41d distcoeff,
     const std::vector<std::vector<CloudPoint> >& pnt_clouds,
     const std::vector<cv::Matx34d>& camera_poses,
     const int range)
{
    if(camera_matx == cv::Matx33d::zeros())
    {
        SfM_ERROR("camera_matx == 0 in BundleAdjustor init");
        return false;
    }
    _camera_matx = camera_matx;
    _distcoeff = distcoeff;

    _pnt_clouds = std::vector<std::vector<CloudPoint> >(pnt_clouds.size(), std::vector<CloudPoint>());

    for(int i = 0; i < pnt_clouds.size(); i++)
    {
        for(int j = 0; j < pnt_clouds[i].size(); j++)
        {
            _pnt_clouds[i].push_back(pnt_clouds[i][j]);
        }
    }

    for(int i = 0; i < camera_poses.size(); i++)
    {
        _camera_poses.push_back(camera_poses[i]);
    }
    _range = -1;

    _init_allready = true;
    _ba_allready = false;

    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn  bool GetImagepntsVSandPnt3D
*
* \brief    compute the image viewable array, and 3d point array, and 2d image array for cv::ba
*           pipeline in my thesis
*
* \param[in]    pnts_num --> total points num
*               views_num --> image num
*               pnt_clous --> point clouds
*               range --> deep search range default = 1
*
* \param[out]   image_pnts --> the corresponding 2d points in image
*               objpnts_vs --> viewable array
*               object_pnts --> 3d points from pnt_clouds
*
* \return   if seccessfult return true, otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool CVBundleAdjustor::
GetImagepntsVSandPnt3D(const int64 pnts_num,
                       const unsigned int views_num,
                       const std::vector<std::vector<CloudPoint> >& pnt_clouds,
                       std::vector<std::vector<cv::Point2d> >& image_pnts,
                       std::vector<std::vector<int> >& objpnts_vs,
                       std::vector<cv::Point3d>& object_pnts,
                       const int range)
{
    if(pnts_num <= 0 || views_num <= 0 || pnt_clouds.empty() || range < -1)
    {
        SfM_ERROR("input parameters are/is not right in BundleAdjustor::GetImagepntsVSandPnt3D");
        return false;
    }

#if DEBUG
    SfM_SENDMSG("parameters are fine, doing GetImagepntsVSandPnt3D");
#endif

    object_pnts = std::vector<cv::Point3d>(pnts_num);
    image_pnts = std::vector<std::vector<cv::Point2d> >(views_num, std::vector<cv::Point2d>(pnts_num));
    objpnts_vs = std::vector<std::vector<int> >(views_num, std::vector<int>(pnts_num, 0));

#if DEBUG
    SfM_SENDMSG("memery malloc are fine in GetImagepntsVSandPnt3D");
#endif
    const int cloud_num = pnt_clouds.size();
    unsigned int object_pnts_num = 0;   // position of object pnt in object_pnts
    for(int i = 0; i < pnt_clouds.size(); i++)  // find through pnt_clouds (all views)
    {
        for(int j = 0; j < pnt_clouds[i].size(); j++) // find through every pnt_cloud  (every views)
        {
            /* get object pnt */
            CloudPoint cloud_pnt = (pnt_clouds[i])[j];  // store object_pnt
            object_pnts[object_pnts_num] = cloud_pnt.object_pnt;

            /* compute visibilities and cooresponding image points */
            (objpnts_vs[i])[object_pnts_num] = 1;  // can be seen in view i
            (image_pnts[i])[object_pnts_num] = cloud_pnt.image_pnt_pre;
            if(i+1 < views_num)
            {
                (objpnts_vs[i+1])[object_pnts_num] = 1; // if true, can be seen in view i+1
                (image_pnts[i+1])[object_pnts_num] = cloud_pnt.image_pnt_nxt;
            }
            /* find the vs in next pnt-clouds */
            bool stop = false; int cloud_step_plus = 0; cv::Point2d local_nxt_reference = cloud_pnt.image_pnt_nxt;
            int most_plus_steps = 0; if(range == -1) most_plus_steps = cloud_num; else most_plus_steps = i+range;
            while(!stop)    // front direction
            {
                cloud_step_plus++;
                if((i+cloud_step_plus) >= std::min<int>(cloud_num, most_plus_steps))    // if i+2 or more > nums of views out
                {
                    stop = true;
                }
                else
                {
                    bool out_key = true;
                    for(int jj = 0; jj < pnt_clouds[i+cloud_step_plus].size(); jj++)
                    {
                        if(local_nxt_reference == (pnt_clouds[i+cloud_step_plus])[jj].image_pnt_pre)   //
                        {
                            (objpnts_vs[i+cloud_step_plus+1])[object_pnts_num] = 1; // if true, can be seen in view i+more
                            (image_pnts[i+cloud_step_plus+1])[object_pnts_num] = (pnt_clouds[i+cloud_step_plus])[jj].image_pnt_nxt;
                            local_nxt_reference = (pnt_clouds[i+cloud_step_plus])[jj].image_pnt_nxt; // reset reference
                            cloud_step_plus++; // go to next view
                            out_key = false;
                            break;
                        }
                    }
                    if(out_key == true) // if not find same point in this cloud, stop
                    {
                        stop = true;
                    }   // end jj
                }
            }   // end nxt find
            /* find the vs in pre pnt-clouds */
            stop = false; int cloud_step_reduce = 0; cv::Point2d local_pre_reference = cloud_pnt.image_pnt_pre;
            int most_reduce_steps = 0; if(range == -1) most_reduce_steps = 0; else most_reduce_steps = i-range;
            while(!stop)    // front direction
            {
                cloud_step_reduce++;
                if((i-cloud_step_reduce) < std::max<int>(0, most_reduce_steps))    // if i+2 or more > nums of views out
                {
                    stop = true;
                }
                else
                {
                    bool out_key = true;
                    for(int jj = 0; jj < pnt_clouds[i-cloud_step_reduce].size(); jj++)
                    {
                        if(local_pre_reference == (pnt_clouds[i-cloud_step_reduce])[jj].image_pnt_nxt)
                        {
                            (objpnts_vs[i-cloud_step_reduce])[object_pnts_num] = 1; // if true, can be seen in view i+more
                            (image_pnts[i-cloud_step_reduce])[object_pnts_num] = (pnt_clouds[i-cloud_step_reduce])[jj].image_pnt_pre;
                            local_pre_reference = (pnt_clouds[i-cloud_step_reduce])[jj].image_pnt_pre; // reset reference
                            cloud_step_reduce++; // go to next view
                            out_key = false;
                            break;
                        }
                    }   // end jj
                    if(out_key == true) // if not find same point in this cloud, stop
                    {
                        stop = true;
                    }
                }
            }   // end pre find
            object_pnts_num++;
        }   // end j
    }   // end i
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn  bool GetNewPntCloudsAndCameraPoses
*
* \brief    get good key points from matches good
*
* \param[out]   pnt_clouds --> container for point clouds, who have been done with bundle adjustment
*               camera_poses --> container for camera poses, who have been done with bundle adjustment
*
* \return   if seccessfult return true, otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool CVBundleAdjustor::
GetNewPntCloudsAndCameraPoses(std::vector<std::vector<CloudPoint> >& pnt_clouds,
                              std::vector<cv::Matx34d>& camera_poses)
{
    if(!_ba_allready)
        return false;
    pnt_clouds = std::vector<std::vector<CloudPoint> >(pnt_clouds.size(), std::vector<CloudPoint>());

    for(int i = 0; i < _pnt_clouds.size(); i++)
    {
        for(int j = 0; j < _pnt_clouds[i].size(); j++)
        {
            pnt_clouds[i].push_back(_pnt_clouds[i][j]);
        }
    }

    camera_poses.clear();
    for(int i = 0; i < _camera_poses.size(); i++)
    {
        camera_poses.push_back(_camera_poses[i]);
    }
    return true;
}
