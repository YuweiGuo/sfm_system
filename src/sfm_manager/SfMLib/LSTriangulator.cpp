#include "LSTriangulator.h"

/* ------------------------------------------------------------------------- */
/* \fn cv::Mat_<double>  LinearLSTriangulation
*
* \brief        do triangulation for two 2D image points with poses. use linear equation system
*
* \param[in]    image_pnt_pre --> form is [a,b,1] from image at time 1
*               image_pnt_nxt --> form is [a,b,1] from image at time 2
*               camera_pose_pre --> camera position of image at time 1
*               camera_pose_nxt --> camera position of image at time 2
*
* \return       the triangulated 3D points
*/
/* ------------------------------------------------------------------------- */
cv::Mat_<double> LSTriangulator::
LinearLSTriangulation(const cv::Point3d image_pnt_pre, const cv::Matx34d camera_pose_pre,
                      const cv::Point3d image_pnt_nxt, const cv::Matx34d camera_pose_nxt)
{
    /* build matrix A for homogenous equation system Ax = 0 */
    /* assume X = (x,y,z,1), for Linear-LS method */
    cv::Point3d u1 = image_pnt_pre;     cv::Point3d u2 = image_pnt_nxt;
    cv::Matx34d p1 = camera_pose_pre;    cv::Matx34d p2 = camera_pose_nxt;

    cv::Matx43d A(u1.x*p1(2,0)-p1(0,0),	u1.x*p1(2,1)-p1(0,1),   u1.x*p1(2,2)-p1(0,2),
                  u1.y*p1(2,0)-p1(1,0),	u1.y*p1(2,1)-p1(1,1),	u1.y*p1(2,2)-p1(1,2),
                  u2.x*p2(2,0)-p2(0,0), u2.x*p2(2,1)-p2(0,1),	u2.x*p2(2,2)-p2(0,2),
                  u2.y*p2(2,0)-p2(1,0), u2.y*p2(2,1)-p2(1,1),	u2.y*p2(2,2)-p2(1,2));
    cv::Matx41d B(-(u1.x*p1(2,3)-p1(0,3)),
                  -(u1.y*p1(2,3)-p1(1,3)),
                  -(u2.x*p2(2,3)-p2(0,3)),
                  -(u2.y*p2(2,3)-p2(1,3)));

    cv::Mat_<double> X;
    cv::solve(A, B, X, cv::DECOMP_SVD);
    return X;
}

/* ------------------------------------------------------------------------- */
/* \fn cv::Mat_<double>  IterativeLinearLSTriangulation
*
* \brief        do triangulation for two 2D image points with poses. use a interative methode
*
* \param[in]    image_pnt_pre --> form is [a,b,1] from image at time 1
*               image_pnt_nxt --> form is [a,b,1] from image at time 2
*               camera_pose_pre --> camera position of image at time 1
*               camera_pose_nxt --> camera position of image at time 2
*
* \return       the triangulated 3D points
*/
/* ------------------------------------------------------------------------- */
cv::Mat_<double> LSTriangulator::
IterativeLinearLSTriangulation(const cv::Point3d image_pnt_pre, const cv::Matx34d camera_pose_pre,
                               const cv::Point3d image_pnt_nxt, const cv::Matx34d camera_pose_nxt)
{
    cv::Point3d u1 = image_pnt_pre;     cv::Point3d u2 = image_pnt_nxt;
    cv::Matx34d p1 = camera_pose_pre;    cv::Matx34d p2 = camera_pose_nxt;

    double wi1 = 1, wi2 = 1;
    cv::Mat_<double> X(4,1);
    cv::Mat_<double> X_ = LinearLSTriangulation(u1, p1, u2, p2);
    X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
    for (int i = 0; i < 10; i++)    //Hartley suggests 10 iterations at most
    {
        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(p1).row(2)*X)(0);   //recalculate weights
        double p2x2 = cv::Mat_<double>(cv::Mat_<double>(p2).row(2)*X)(0);

        if(std::fabs(wi1 - p2x1) <= EPSILON && std::fabs(wi2 - p2x2) <= EPSILON) break; //breaking point

        wi1 = p2x1;
        wi2 = p2x2;

        cv::Matx43d A((u1.x*p1(2,0)-p1(0,0))/wi1,   (u1.x*p1(2,1)-p1(0,1))/wi1,     (u1.x*p1(2,2)-p1(0,2))/wi1, //reweight equations and solve
                      (u1.y*p1(2,0)-p1(1,0))/wi1,	(u1.y*p1(2,1)-p1(1,1))/wi1,     (u1.y*p1(2,2)-p1(1,2))/wi1,
                      (u2.x*p2(2,0)-p2(0,0))/wi2,   (u2.x*p2(2,1)-p2(0,1))/wi2,     (u2.x*p2(2,2)-p2(0,2))/wi2,
                      (u2.y*p2(2,0)-p2(1,0))/wi2,   (u2.y*p2(2,1)-p2(1,1))/wi2,     (u2.y*p2(2,2)-p2(1,2))/wi2);
        cv::Matx41d B(-(u1.x*p1(2,3)-p1(0,3))/wi1,
                      -(u1.y*p1(2,3)-p1(1,3))/wi1,
                      -(u2.x*p2(2,3)-p2(0,3))/wi2,
                      -(u2.y*p2(2,3)-p2(1,3))/wi2);


        cv::solve(A, B, X_, cv::DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
    }
    return X;
}

/* ------------------------------------------------------------------------- */
/* \fn double  DoTriangulation
*
* \brief        do triangulation for more keypoints pairs. _keypnts_new_pre and _keypnts_new_nxt
*
* \return       return the average reprojection error
*/
/* ------------------------------------------------------------------------- */
double LSTriangulator::
DoTriangulation()
{
    std::vector<cv::Point2d> pnts_new_pre; HelpFunc::KeyPntsToPnts(_keypnts_new_pre, pnts_new_pre);
    std::vector<cv::Point2d> pnts_new_nxt; HelpFunc::KeyPntsToPnts(_keypnts_new_nxt, pnts_new_nxt);

    if(!_init_allready)
        SfM_ERROR("Triangulator has been not init, you will get white pnt_cloud!");
    cv::Matx33d cmatx = _camera_matx;   cv::Matx33d cmatx_inv = _camera_matx.inv();
    cv::Matx34d p1 = _camera_pose_pre;  cv::Matx34d p2 = _camera_pose_nxt;

    std::vector<CloudPoint> pnt_cloud;

    SfM_SENDMSG("Triangulating...");

    double time = cv::getTickCount();
    std::vector<double> reproj_error;
    unsigned int pts_size = pnts_new_pre.size();

    cv::Matx34d cp1 = cmatx * p1;
    cv::Matx34d cp2 = cmatx * p2;


    #pragma omp parallel for num_threads(1)
    for (int i = 0; i < pts_size; i++)
    {
        /* get homogenous vector of keypnt 1 */
        cv::Point2d kp1 = pnts_new_pre[i];
        cv::Point3d u1(kp1.x, kp1.y, 1.0);
        cv::Mat_<double> uv1 = cv::Mat_<double>(cmatx_inv) * cv::Mat_<double>(u1);
        u1.x = uv1(0); u1.y = uv1(1); u1.z = uv1(2);

        /* get homogenous vector of keypnt 2 */
        cv::Point2d kp2 = pnts_new_nxt[i];
        cv::Point3d u2(kp2.x, kp2.y, 1.0);
        cv::Mat_<double> uv2 = cv::Mat_<double>(cmatx_inv) * cv::Mat_<double>(u2);
        u2.x = uv2(0); u2.y = uv2(1); u2.z = uv2(2);

        /* do triangulation for 2 point */
        cv::Mat_<double> X = IterativeLinearLSTriangulation(u1, p1, u2, p2);
        cv::Mat_<double> xPt_img1 = cv::Mat_<double>(cp1) * X;				//reproject
        cv::Mat_<double> xPt_img2 = cv::Mat_<double>(cp2) * X;				//reproject

        cv::Point2d xPt_img1_(xPt_img1(0)/xPt_img1(2),xPt_img1(1)/xPt_img1(2));
        cv::Point2d xPt_img2_(xPt_img2(0)/xPt_img2(2),xPt_img2(1)/xPt_img2(2));

        double error1 = cv::norm(xPt_img1_-kp1);
        double error2 = cv::norm(xPt_img2_-kp2);

        #pragma omp critical
        {
            reproj_error.push_back(error2);

            CloudPoint cp;
            cp.object_pnt = cv::Point3d(X(0), X(1), X(2));
            cp.error_pre = error1;
            cp.error_nxt = error2;
            cp.image_pnt_pre = kp1;
            cp.image_pnt_nxt = kp2;

            if(_init_allready)
            {
                if(_image_pre.type() >= 23) //RGBA
                {
                    cv::Vec3b kp1_color(_image_pre.at<cv::Vec4b>(kp1).val[0], _image_pre.at<cv::Vec4b>(kp1).val[1], _image_pre.at<cv::Vec4b>(kp1).val[2]);
                    cv::Vec3b kp2_color(_image_nxt.at<cv::Vec4b>(kp2).val[0], _image_nxt.at<cv::Vec4b>(kp2).val[1], _image_nxt.at<cv::Vec4b>(kp2).val[2]);
                    cp.pnt_color = 0.5*(kp1_color + kp2_color);
                }
                if(_image_pre.type() >= 16 && _image_pre.type() <= 22)  //RGB
                {
                    cp.pnt_color = 0.5*(_image_pre.at<cv::Vec3b>(kp1)+_image_nxt.at<cv::Vec3b>(kp2));
                }
                if(_image_pre.type() <= 6)  //GRAY
                {
                    int color = 0.5*(_image_pre.at<int>(kp1)+_image_nxt.at<int>(kp2));
                    cp.pnt_color.val[0] = color; cp.pnt_color.val[1] = color; cp.pnt_color.val[2] = color;
                }
            }
            else
                cp.pnt_color = cv::Vec3b(255, 255, 255);

            if(error1 < 50.0 && error2 < 50.0)
                pnt_cloud.push_back(cp);

        }
    }
    cv::Scalar mse = cv::mean(reproj_error);
    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    SfM_SENDMSG("Done. (" + std::to_string(pnt_cloud.size())+
                   " points, " + std::to_string(time) +
                   "s, mean reproj err for image_nxt = " + std::to_string(mse[0]));
    _triangulation_allready = true;
    StorePointCloud(pnt_cloud);
    return mse[0];
}

/* ------------------------------------------------------------------------- */
/* \fn bool  UpdateImage
*
* \brief        we can use triangulator without init, but the points have only white color.
*               we use this function to update images for the calculated pnt_clouds.
*               then the points in clouds will have corresponding color value
*
* \param[in]    image_pre --> the image at time 1 for pnt_cloud
*               image_nxt --> the image at time 2 for pnt_cloud
*
* \return       if parameters are error return false, otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool LSTriangulator::
UpdateImage(const cv::Mat image_pre, const cv::Mat image_nxt)
{
    if(!_triangulation_allready)
    {
        SfM_ERROR("thie method can only be used, when you have computed the point cloud, you can use Init()+DoTriangulation() to make point cloud");
        return false;
    }
    if(image_pre.empty() || image_nxt.empty())
    {
        SfM_ERROR("image_pre or _nxt has no data");
        return false;
    }

    _image_pre = image_pre.clone();    _image_nxt = image_nxt.clone();
    SfM_SENDMSG("Triangulator Init seccsessfully");
    _init_allready = true;

    std::vector<CloudPoint> pnt_cloud; GetPointCloud(pnt_cloud);

    for(int i = 0; i < pnt_cloud.size(); i++)
    {
        cv::Point2d kp1 = pnt_cloud[i].image_pnt_pre; cv::Point2d kp2 = pnt_cloud[i].image_pnt_nxt;
        if(_image_pre.type() >= 16 && _image_pre.type() <= 22)  //RGB
        {
            pnt_cloud[i].pnt_color = 0.5*(_image_pre.at<cv::Vec3b>(kp1)+_image_nxt.at<cv::Vec3b>(kp2));
        }
        if(_image_pre.type() <= 6)  //GRAY
        {
            int color = 0.5*(_image_pre.at<int>(kp1)+_image_nxt.at<int>(kp2));
            pnt_cloud[i].pnt_color.val[0] = color; pnt_cloud[i].pnt_color.val[1] = color; pnt_cloud[i].pnt_color.val[2] = color;
        }
    }
    StorePointCloud(pnt_cloud);
    return true;
}


/* ------------------------------------------------------------------------- */
/* \fn bool  Init
*
* \brief        init the parameters members in class
*
* \param[in]    image_pre --> the image at time 1
*               image_nxt --> the image at time 2
*               keypnts_new_pre --> keypoints in the image at time 1
*               keypnts_new_nxt --> keypoints in the image at time 2
*               camera_matx --> camera matx
*               camera_pose_pre --> camera position for image at time 1
*               camera_pose_nxt --> camera position for image at time 2
*
* \return       if parameters are error return false, otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool LSTriangulator::
Init(const cv::Mat image_pre, const cv::Mat image_nxt,
     const std::vector<cv::KeyPoint>& keypnts_new_pre, const std::vector<cv::KeyPoint>& keypnts_new_nxt,
     const cv::Matx33d& camera_matx, const cv::Matx34d& camera_pose_pre, const cv::Matx34d& camera_pose_nxt)
{
    if(!image_pre.empty() && !image_nxt.empty())
    {
        _image_pre = image_pre.clone();    _image_nxt = image_nxt.clone();
        _init_allready = true;
    }
    else
    {
        _init_allready = false;
        SfM_ERROR("error in Triangulator Init, no data in image, _pnt_cloud has only white color, init will go on");
    }

    _keypnts_new_pre = keypnts_new_pre; _keypnts_new_nxt = keypnts_new_nxt;
    _camera_matx = camera_matx;
    _camera_pose_pre = camera_pose_pre; _camera_pose_nxt = camera_pose_nxt;

    SfM_SENDMSG("Triangulator Init seccsessfully");
    _triangulation_allready = false;
    return true;
}

