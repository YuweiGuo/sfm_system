#include "SfMLibCameraPoseComputer.h"

/* ------------------------------------------------------------------------- */
/* \fn bool GetKeyPntsNew
*
* \brief    get keypnts from member parameters
*
* \param[out]       keypnts_good_pre --> container for the new keypnts in image at time 1
*                   keypnts_good_nxt --> container for the new keypnts in image at time 2
*
* \return   if keypoint is empty return false, otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool SfMLibCameraPoseComputer::
GetKeyPntsNew(std::vector<cv::KeyPoint>& keypnts_new_pre,
              std::vector<cv::KeyPoint>& keypnts_new_nxt)
{
    keypnts_new_pre.clear(); keypnts_new_nxt.clear();
    for(int i = 0; i < _keypnts_new_pre.size(); i++)
    {
        keypnts_new_pre.push_back(_keypnts_new_pre[i]);
        keypnts_new_nxt.push_back(_keypnts_new_nxt[i]);
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool GetKeyPntsNew
*
* \brief    get matches from member parameters
*
* \param[out]       matches_new --> container for the new matches
*
* \return   if matches is empty return false, otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool SfMLibCameraPoseComputer::
GetMatchesNew(std::vector<cv::DMatch>& matches_new)
{
    matches_new.clear();
    for(int i = 0; i < _matches_new.size(); i++)
    {
        matches_new.push_back(matches_new[i]);
    }
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool GetCameraPose
*
* \brief    get camera pose nxt from member parameters
*
* \param[out]       camera_pose --> container for the camera pose
*
* \return   return true
*/
/* ------------------------------------------------------------------------- */
bool SfMLibCameraPoseComputer::
GetCameraPose(cv::Matx34d& camera_pose)
{
    camera_pose = _camera_pose_nxt;
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn void StoreKeyPntsMatchesNew
*
* \brief    store keypnts and matches in parameter members
*
* \param[in]        keypnts_new_pre --> keypnts new in image at time 1, who want to be stored
*                   keypnts_new_nxt --> keypnts new in image at time 2, who want to be stored
*                   matches_new --> new matches, who want to be stored
*/
/* ------------------------------------------------------------------------- */
void SfMLibCameraPoseComputer::
StoreKeyPntsMatchesNew(const std::vector<cv::KeyPoint> keypnts_new_pre,
                       const std::vector<cv::KeyPoint> keypnts_new_nxt,
                       const std::vector<cv::DMatch> matches_new)
{
    _keypnts_new_pre.clear(); _keypnts_new_nxt.clear(); _matches_new.clear();
    for(int i = 0; i < keypnts_new_pre.size(); i++)
    {
        _keypnts_new_pre.push_back(keypnts_new_pre[i]);
        _keypnts_new_nxt.push_back(keypnts_new_nxt[i]);
        _matches_new.push_back(matches_new[i]);
    }
}

/* ------------------------------------------------------------------------- */
/* \fn cv::Mat_<double>  ComputeFundamentalMatx
*
* \brief    compute fundamental matx with keypnts and matchs
*
* \param[in]        keypnts_good_pre --> keypnts in image at time 1, they are resources for compute
*                   keypnts_good_nxt --> keypnts in image at time 2, they are resources for compute
*                   matches_good --> good matches, they are resources for compute
*
* \param[out]       keypnts_new_pre --> keypnts_new in image at time 1, they are got from compute process
*                   keypnts_new_nxt --> keypnts_new in image at time 2, they are got from compute process
*                   matches_new --> new matches, they are got from compute process
*
* \return       return the fundamentalmatrix
*/
/* ------------------------------------------------------------------------- */
cv::Mat_<double> SfMLibCameraPoseComputer::
ComputeFundamentalMatx(const std::vector<cv::KeyPoint> keypnts_good_pre,
                       const std::vector<cv::KeyPoint> keypnts_good_nxt,
                       const std::vector<cv::DMatch> matches_good,
                       std::vector<cv::KeyPoint>& pnts_new_pre,
                       std::vector<cv::KeyPoint>& pnts_new_nxt,
                       std::vector<cv::DMatch>& matches_new)
{
    /* keypnts to pnts*/
    std::vector<cv::Point2d> pnts_good_pre; HelpFunc::KeyPntsToPnts(keypnts_good_pre, pnts_good_pre);
    std::vector<cv::Point2d> pnts_good_nxt; HelpFunc::KeyPntsToPnts(keypnts_good_nxt, pnts_good_nxt);
    if(pnts_good_pre.empty() || pnts_good_nxt.empty())
    {
        SfM_ERROR("pnts_good_pre/nxt in ComputeFundamentalMatx is/are empty");
        return cv::Mat_<double>::zeros(0, 0);
    }

    /* compute fundamental matrix */
    std::vector<uchar> status(pnts_good_pre.size());
    double min_val, max_val;
    cv::minMaxIdx(pnts_good_pre, &min_val, &max_val);
    cv::Mat_<double> FMatx = cv::findFundamentalMat(pnts_good_pre, pnts_good_nxt, cv::FM_RANSAC, 0.006 * max_val, 0.99, status);

    /* get new super good on line pnts */
    for(int i = 0; i < status.size(); i++)
    {
        if(status[i])
        {
            pnts_new_pre.push_back(keypnts_good_pre[i]);
            pnts_new_nxt.push_back(keypnts_good_nxt[i]);
            matches_new.push_back(matches_good[i]);
        }
    }
    return FMatx;
}

/* ------------------------------------------------------------------------- */
/* \fn void StoreCameraPoseNxt
*
* \brief    store pose_nxt in parameter members
*
* \param[in]        camera_pose_nxt --> the new pose_nxt, who want to be stored

*/
/* ------------------------------------------------------------------------- */
void SfMLibCameraPoseComputer::
StoreCameraPoseNxt(const cv::Matx34d camera_pose_nxt)
{
    _camera_pose_nxt = camera_pose_nxt;
    return;
}
