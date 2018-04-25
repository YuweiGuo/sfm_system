#include "SfMLibFeatureMatcher.h"

/* ------------------------------------------------------------------------- */
/* \fn bool GetGoodKeyPnts
*
* \brief    get key points from member parameters
*
* \param[out]       keypnts_good_pre --> container for key points from images at time 1
*                   keypnts_good_nxt --> container for key points from images at time 2
*
* \return       if keypnts are empty return false, otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool SfMLibFeatureMatcher::GetGoodKeyPnts(std::vector<cv::KeyPoint> &keypnts_good_pre, std::vector<cv::KeyPoint> &keypnts_good_nxt)
{
    keypnts_good_pre.clear();   keypnts_good_nxt.clear();
    if(_keypnts_good_pre.empty() || _keypnts_good_nxt.empty())
        return false;
    keypnts_good_pre = _keypnts_good_pre;
    keypnts_good_nxt = _keypnts_good_nxt;
    return true;
}
/* ------------------------------------------------------------------------- */
/* \fn bool GetGoodMachtes
*
* \brief    get matches from member parameters
*
* \param[out]       matches_good --> container for machtes
*
* \return       if matches are empty return false, otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool SfMLibFeatureMatcher::GetGoodMachtes(std::vector<cv::DMatch> &matches_good)
{
    if(_matches_good.empty())
        return false;
    matches_good.clear();
    matches_good = _matches_good;
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn void StoreGoodKeyPnts
*
* \brief    store keypnts in member parameters
*
* \param[in]        keypnts_good_pre --> the keypnts in image at time 1, who want to be stored
*                   keypnts_good_nxt --> the keypnts in image at time 2, who want to be stored
*/
/* ------------------------------------------------------------------------- */
void SfMLibFeatureMatcher::
StoreGoodKeyPnts(const std::vector<cv::KeyPoint> &keypnts_good_pre, const std::vector<cv::KeyPoint> &keypnts_good_nxt)
{
    _keypnts_good_pre.clear(); _keypnts_good_nxt.clear();
    _keypnts_good_pre = keypnts_good_pre;
    _keypnts_good_nxt = keypnts_good_nxt;
}

/* ------------------------------------------------------------------------- */
/* \fn void StoreGoodMachtes
*
* \brief    store matches in member parameters
*
* \param[in]        matches_good --> the matches, who want to be stored
*/
/* ------------------------------------------------------------------------- */
void SfMLibFeatureMatcher::StoreGoodMachtes(const std::vector<cv::DMatch> &matches_good)
{
    _matches_good.clear();
    _matches_good = matches_good;
}
