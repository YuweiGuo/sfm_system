#include "EPGCameraPoseComputer.h"

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
*                   epg_firstlength --> the move unit in this step
*
* \return       if not parameters are false, renturn false. otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool EPGCameraPoseComputer::
Init(const std::vector<cv::KeyPoint> keypnts_good_pre,
     const std::vector<cv::KeyPoint> keypnts_good_nxt,
     const std::vector<cv::DMatch> matches_good,
     const cv::Matx33d camera_matx,
     const cv::Matx41d distcoeff,
     const float &epg_firstlength)
{
    if(keypnts_good_pre.empty() || keypnts_good_nxt.empty() || matches_good.empty() || camera_matx == cv::Matx33d::zeros())
    {
        SfM_ERROR("the init data for EPGCameraPoseComputer is empty, or camera_matx is 0");
        return false;
    }
    if(keypnts_good_pre.size() != keypnts_good_nxt.size() || keypnts_good_pre.size() != matches_good.size())
    {
        SfM_ERROR("the init data for EPGCameraPoseComputer is failer, size of keypnts or/and matches are not same");
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
    _init_allready = true;
    _compute_allready = false;
    _epg_firstlength = epg_firstlength;
    SfM_SENDMSG("EPGCameraPoseComputer Init4EM seccessful");
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool DoCameraPoseCompute
*
* \brief        the virtual function will be here implemented.
*               compute camera poses using epipolar geometry method
*               pipeline in my thesis
*/
/* ------------------------------------------------------------------------- */
bool EPGCameraPoseComputer::
DoCameraPoseCompute()
{
    if(!_init_allready)
    {
        SfM_ERROR("EPGCameraPoseComputer has been not init, specialy for essential methed if you want to use");
        return false;
    }
    SfM_SENDMSG("computing camera-pos with essential-matrix ...");
    double time = cv::getTickCount();

    /* fundamental matrix and essential matrix */
    std::vector<cv::KeyPoint> keypnts_new_pre, keypnts_new_nxt;
    std::vector<cv::DMatch> matches_new;
    cv::Mat_<double> FMatx = ComputeFundamentalMatx(_keypnts_good_pre, _keypnts_good_nxt, _matches_good, keypnts_new_pre, keypnts_new_nxt, matches_new);
    StoreKeyPntsMatchesNew(keypnts_new_pre, keypnts_new_nxt, matches_new);
    cv::Mat_<double> EMatx = cv::Mat_<double>(_camera_matx.t()) * FMatx * cv::Mat_<double>(_camera_matx);
    if(std::fabs(cv::determinant(EMatx)) >= EPSILON)
    {
        SfM_ERROR("error in camera pos computing with e-matx, det(e) != 0");
        return false;
    }

    /* find right R T combination */
    cv::Mat_<double> R1(3,3);
    cv::Mat_<double> R2(3,3);
    cv::Mat_<double> T1(1,3);
    cv::Mat_<double> T2(1,3);

    LSTriangulator triangulator;   // for test
    cv::Matx34d p_nxt;
    {
        if (!DecomposeEMatxtoRandT(EMatx, R1, R2, T1, T2))
        {
            SfM_ERROR("failt to Decompose EMatx to R and T");
            return false;
        }
        if(cv::determinant(R1)+1.0 < 1e-09)
        {
            SfM_SENDMSG("det(R) == -1 ["+std::to_string(determinant(R1))+"]: flip E's sign");
            EMatx = -EMatx;
            DecomposeEMatxtoRandT(EMatx, R1, R2, T1, T2);
        }
        if (!CheckCoherentRotation(R1))
        {
            SfM_ERROR("resulting rotation is not coherent");
            return false;
        }
        cv::Matx34d p_pre = cv::Matx34d(1, 0, 0, 0,
                                        0, 1, 0, 0,
                                        0, 0, 1, 0);
        p_nxt = cv::Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	T1(0),
                            R1(1,0),	R1(1,1),	R1(1,2),	T1(1),
                            R1(2,0),	R1(2,1),	R1(2,2),	T1(2));

        std::vector<CloudPoint> pnt_cloud_1, pnt_cloud_2;
        triangulator.Init(cv::Mat::zeros(0,0,0), cv::Mat::zeros(0,0,0), keypnts_new_pre, keypnts_new_nxt, _camera_matx, p_pre, p_nxt);
        double reproj_error1 = triangulator.DoTriangulation();
        triangulator.GetPointCloud(pnt_cloud_1);

        triangulator.Init(cv::Mat::zeros(0,0,0), cv::Mat::zeros(0,0,0), keypnts_new_nxt, keypnts_new_pre, _camera_matx, p_nxt, p_pre);
        double reproj_error2 = triangulator.DoTriangulation();
        triangulator.GetPointCloud(pnt_cloud_2);

        std::vector<uchar> tmp_status;

        SfM_SENDMSG("testing p_nxt :");
        SfM_SENDMSG(p_nxt);

        //check if pointa are triangulated --in front-- of cameras for all 4 kombinations
        if (!TestTriangulation(pnt_cloud_1, p_nxt, tmp_status) || !TestTriangulation(pnt_cloud_2, p_pre, tmp_status) ||
                reproj_error1 > 100.0 || reproj_error2 > 100.0)
        {
            p_nxt = cv::Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	T2(0),
                                R1(1,0),	R1(1,1),	R1(1,2),	T2(1),
                                R1(2,0),	R1(2,1),	R1(2,2),	T2(2));

            pnt_cloud_1.clear(); pnt_cloud_2.clear();
            triangulator.Init(cv::Mat::zeros(0,0,0), cv::Mat::zeros(0,0,0), keypnts_new_pre, keypnts_new_nxt, _camera_matx, p_pre, p_nxt);
            reproj_error1 = triangulator.DoTriangulation();
            triangulator.GetPointCloud(pnt_cloud_1);

            triangulator.Init(cv::Mat::zeros(0,0,0), cv::Mat::zeros(0,0,0), keypnts_new_nxt, keypnts_new_pre, _camera_matx, p_nxt, p_pre);
            reproj_error2 = triangulator.DoTriangulation();
            triangulator.GetPointCloud(pnt_cloud_2);


            SfM_SENDMSG("testing p_nxt :");
            SfM_SENDMSG(p_nxt);

            if (!TestTriangulation(pnt_cloud_1, p_nxt, tmp_status) || !TestTriangulation(pnt_cloud_2, p_pre, tmp_status) ||
                    reproj_error1 > 100.0 || reproj_error2 > 100.0)
            {
                if (!CheckCoherentRotation(R2))
                {
                    SfM_ERROR("resulting rotation is not coherent");
                    return false;
                }
                p_nxt = cv::Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	T1(0),
                                    R2(1,0),	R2(1,1),	R2(1,2),	T1(1),
                                    R2(2,0),	R2(2,1),	R2(2,2),	T1(2));

                pnt_cloud_1.clear(); pnt_cloud_2.clear();
                triangulator.Init(cv::Mat::zeros(0,0,0), cv::Mat::zeros(0,0,0), keypnts_new_pre, keypnts_new_nxt, _camera_matx, p_pre, p_nxt);
                reproj_error1 = triangulator.DoTriangulation();
                triangulator.GetPointCloud(pnt_cloud_1);

                triangulator.Init(cv::Mat::zeros(0,0,0), cv::Mat::zeros(0,0,0), keypnts_new_nxt, keypnts_new_pre, _camera_matx, p_nxt, p_pre);
                reproj_error2 = triangulator.DoTriangulation();
                triangulator.GetPointCloud(pnt_cloud_2);


                SfM_SENDMSG("testing p_nxt :");
                SfM_SENDMSG(p_nxt);

                if (!TestTriangulation(pnt_cloud_1, p_nxt, tmp_status) || !TestTriangulation(pnt_cloud_2, p_pre, tmp_status) ||
                     reproj_error1 > 100.0 || reproj_error2 > 100.0)
                {
                    p_nxt = cv::Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	T2(0),
                                        R2(1,0),	R2(1,1),	R2(1,2),	T2(1),
                                        R2(2,0),	R2(2,1),	R2(2,2),	T2(2));

                    pnt_cloud_1.clear(); pnt_cloud_2.clear();
                    triangulator.Init(cv::Mat::zeros(0,0,0), cv::Mat::zeros(0,0,0), keypnts_new_pre, keypnts_new_nxt, _camera_matx, p_pre, p_nxt);
                    reproj_error1 = triangulator.DoTriangulation();
                    triangulator.GetPointCloud(pnt_cloud_1);

                    triangulator.Init(cv::Mat::zeros(0,0,0), cv::Mat::zeros(0,0,0), keypnts_new_nxt, keypnts_new_pre, _camera_matx, p_nxt, p_pre);
                    reproj_error2 = triangulator.DoTriangulation();
                    triangulator.GetPointCloud(pnt_cloud_2);

                    SfM_SENDMSG("testing p_nxt :");
                    SfM_SENDMSG(p_nxt);

                    if (!TestTriangulation(pnt_cloud_1, p_nxt, tmp_status) || !TestTriangulation(pnt_cloud_2, p_pre, tmp_status) ||
                         reproj_error1 > 100.0 || reproj_error2 > 100.0)
                    {
                        SfM_ERROR("sorry, no R and T combination is richt");
                        return false;
                    }
                }
            }
        }
    }

    if(_epg_firstlength != 1.0)
    {
        cv::Matx33d R; cv::Matx31d T;
        HelpFunc::Matx34ToRT(p_nxt, R, T);
        T = _epg_firstlength*T;
        HelpFunc::RTToMatx34(R, T, p_nxt);
    }
    StoreCameraPoseNxt(p_nxt);

    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    SfM_SENDMSG("Done. (" + std::to_string(time) +"s)");

    _compute_allready = true;
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool DecomposeEMatxtoRandT
*
* \brief        take SVD of essential matrix and get 4 properties from R and T
*
* \param[in]    EMatx --> essential matrix
*
* \param[out}   R1 --> first property for rotation matx
*               R2 --> second property for rotation matx
*               T1 --> first property for translation matx
*               T2 --> second property for translation matx
*
* \return   if sigular values has error return false, otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool EPGCameraPoseComputer::
DecomposeEMatxtoRandT(const cv::Mat_<double> EMatx,
                      cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                      cv::Mat_<double> &T1, cv::Mat_<double> &T2)
{
    //Using HZ E decomposition
    cv::Mat_<double> svd_u, svd_vt, svd_w;
    TakeSVDOfEMatx(EMatx,svd_u,svd_vt,svd_w);

    //check if first and second singular values are the same (as they should be)
    double singular_values_ratio = std::fabs(svd_w.at<double>(0) / svd_w.at<double>(1));
    if(singular_values_ratio > 1.0)
        singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
    if (singular_values_ratio < 0.7) {
        SfM_ERROR("singular values are too far apart\n");
        return false;
    }
    cv::Matx33d W( 0,-1, 0,	//HZ 9.13
                   1, 0, 0,
                   0, 0, 1);
    cv::Matx33d Wt( 0, 1, 0,
                   -1, 0, 0,
                    0, 0, 1);
    R1 = svd_u * cv::Mat(W) * svd_vt; //HZ 9.19
    R2 = svd_u * cv::Mat(Wt) * svd_vt; //HZ 9.19
    T1 = svd_u.col(2); //u3
    T2 = -svd_u.col(2); //u3
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool TakeSVDOfEMatx
*
* \brief        take SVD of essential matrix
*
* \param[in]    EMatx --> essential matrix
*
* \param[out}   svd_u --> the component U in UWVt
*               svd_vt --> the component Vt in UWVt
*               svd_w --> the pomponent W in UWVt
*/
/* ------------------------------------------------------------------------- */
void EPGCameraPoseComputer::
TakeSVDOfEMatx(const cv::Mat_<double> EMatx, cv::Mat_<double> &svd_u, cv::Mat_<double> &svd_vt, cv::Mat_<double> &svd_w)
{
    cv::SVD svd(EMatx, cv::SVD::MODIFY_A);
    svd_u = svd.u;
    svd_vt = svd.vt;
    svd_w = svd.w;
}

/* ------------------------------------------------------------------------- */
/* \fn bool CheckCoherentRotation
*
* \brief        check the correctness of rotation matx, det(R) = 1
*
* \param[in]    R --> target rotation matrx
*
* \return   if right return true , otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool EPGCameraPoseComputer::
CheckCoherentRotation(const cv::Mat_<double> R)
{
    if(std::fabs(determinant(R))-1.0 > 1e-07) {
        SfM_ERROR("det(R) != +-1.0, this is not a rotation matrix");
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool TestTriangulation
*
* \brief       check the reprojection of 3D points
*
* \param[in]    pnt_cloud --> target point cloud
*               pos --> camera position
*
* \param[out]   status --> the meet points num
*
* \return   if > 0.75 meet rate return true , otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool EPGCameraPoseComputer::
TestTriangulation(const std::vector<CloudPoint> pnt_cloud, const cv::Matx34d pos, std::vector<uchar>& status)
{
    std::vector<cv::Point3d> pcloud_pt3d = HelpFunc::PointCloudToPoints(pnt_cloud);
    std::vector<cv::Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());

    cv::Matx44d P4x4 = cv::Matx44d::eye();
    for(int i = 0; i < 12; i++)
    {
        P4x4.val[i] = pos.val[i];
    }
    cv::perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);

    status.resize(pnt_cloud.size(),0);
    for (int i=0; i< pnt_cloud.size(); i++)
    {
        status[i] = (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
    }
    int count = cv::countNonZero(status);

    double percentage = ((double)count / (double)pnt_cloud.size());

    SfM_SENDMSG(std::to_string(percentage*100.0)+"% are in front of camera");

    if(percentage < 0.75)
        return false; //less than 75% of the points are in front of the camera
    return true;
}


