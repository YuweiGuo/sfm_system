#include "SfMLibTriangulator.h"

/* ------------------------------------------------------------------------- */
/* \fn void StorePointCloud
*
* \brief    store pnt cloud in _cloud_pnt
*
* \param[in]    pnt_cloud --> the point cloud, wo want to be stored
*/
/* ------------------------------------------------------------------------- */
void SfMLibTriangulator::StorePointCloud(const std::vector<CloudPoint> &pnt_cloud)
{
    _pnt_cloud.clear();
    _pnt_cloud = pnt_cloud;
}

/* ------------------------------------------------------------------------- */
/* \fn bool SfMLibTriangulator
*
* \brief    get the pnt cloud from _cloud_pnt
*
* \param[out]    pnt_cloud --> the point cloud, wo want to get data
*
* \return   if _pnt_cloud is empty, return false, otherwise return true
*/
/* ------------------------------------------------------------------------- */
bool SfMLibTriangulator::GetPointCloud(std::vector<CloudPoint> &pnt_cloud)
{
    if(_pnt_cloud.empty())
        return false;
    pnt_cloud.clear();
    pnt_cloud = _pnt_cloud;
    return true;
}
