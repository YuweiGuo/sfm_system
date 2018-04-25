#include "viewer.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/* ------------------------------------------------------------------------- */
/* \fn bool Run()
*
* \brief for the outside ros loop call
*/
/* ------------------------------------------------------------------------- */
void VIEWER::
run()
{
    start();
    stop();
}

/* ------------------------------------------------------------------------- */
/* \fn bool Start()
*
* \brief start the ros loop, and run show cloud
*/
/* ------------------------------------------------------------------------- */
void VIEWER::
start()
{
    _spinner.start();
    showCloud();
}

/* ------------------------------------------------------------------------- */
/* \fn bool Start()
*
* \brief stop the ros loop, and run show cloud
*/
/* ------------------------------------------------------------------------- */
void VIEWER::
stop()
{
    _spinner.stop();
}

/* ------------------------------------------------------------------------- */
/* \fn void showCloud()
*
* \brief    this function will setup the pcl visualizer environment.
*           and if get update signal, the cloud pool will get all clouds, who want to show.
*           and then show them
*/
/* ------------------------------------------------------------------------- */
void VIEWER::
showCloud()
{
    /* viewer init */
    _viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(_sfm_cloud);
    _viewer->addPointCloud<pcl::PointXYZRGB> (_sfm_cloud, rgb, _cloudName);
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    _viewer->registerKeyboardCallback(&VIEWER::KeyboardEventOccurred, *this);
    _viewer->initCameraParameters ();
    _viewer->setShowFPS(true);
    _viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);

    while(ros::ok())
    {
        if(_manager_update)
        {
            _lock.lock();
            _manager_update = false;
            _lock.unlock();
            _all_cloud->clear();

            int sfm_num = _sfm_cloud->size();
            for(int i = 0; i < sfm_num && sfm_cloud_show; i++)
                _all_cloud->push_back(_sfm_cloud->at(i));

            int laser_num = _laser_cloud->size();
            for(int i = 0; i < laser_num && laser_cloud_show; i++)
                _all_cloud->push_back(_laser_cloud->at(i));

            int sfm_laser = _amor_positions_4comp.size();
            for(int i = 0; i < sfm_laser && sfm_laser_pose_show; i++)
            {
                pcl::PointXYZRGB pnt;
                pnt.x = _amor_positions_4comp[i].val[0];
                pnt.y = _amor_positions_4comp[i].val[1];
                pnt.z = _amor_positions_4comp[i].val[2];
                pnt.r = 255;
                pnt.g = 255;
                pnt.b = 0;
                _all_cloud->push_back(pnt);
            }

            int sfm_camera = _camera_positions_4comp.size();
            for(int i = 0; i < sfm_camera && sfm_camera_pose_show; i++)
            {
                pcl::PointXYZRGB pnt;
                pnt.x = _camera_positions_4comp[i].val[0];
                pnt.y = _camera_positions_4comp[i].val[1];
                pnt.z = _camera_positions_4comp[i].val[2];
                pnt.r = 0;
                pnt.g = 255;
                pnt.b = 0;
                _all_cloud->push_back(pnt);
            }

            int ba_num = _ba_cloud->size();
            for(int i = 0; i < ba_num && ba_cloud_show; i++)
                _all_cloud->push_back(_ba_cloud->at(i));

            _viewer->updatePointCloud(_all_cloud, _cloudName);
        }
        _viewer->spinOnce(10);
    }
    _viewer->close();
}

/* ------------------------------------------------------------------------- */
/* \fn void SfMManagerSyncCallback()
*
* \brief    this function is for the synchronized msg receive. and then store them in array.
*           in the porcess it will make a filter for the clouds, and for show the coordinate will be changed.
*           if all messages have been received, it will change the update signal. make the new points to show
*
* \param[in]    camera_pose_msg --> camera pose message
*               amor_pose_msg --> amor pose message_filters
*               cloud_msg --> point cloud message
*/
/* ------------------------------------------------------------------------- */
unsigned int rv_num = 0;
void VIEWER::SfMManagerSyncCallback(const geometry_msgs::PoseStampedConstPtr& camera_pose_msg,
                                    const geometry_msgs::PoseStampedConstPtr& amor_pose_msg,
                                    const sensor_msgs::PointCloudConstPtr& cloud_msg)
{
    int counter = 0;
    /* receive sfm pnt cloud data */
    std::vector<pcl::PointXYZRGB> pnt_cloud_temp;
    {
        for(int i = 0; i < cloud_msg->points.size(); i++)
        {
            pcl::PointXYZRGB pcl_pnt;
            pcl_pnt.x = cloud_msg->points[i].x;
            pcl_pnt.y = cloud_msg->points[i].y;
            pcl_pnt.z = cloud_msg->points[i].z;

            pcl_pnt.r = cloud_msg->channels[i].values[0];
            pcl_pnt.g = cloud_msg->channels[i].values[1];
            pcl_pnt.b = cloud_msg->channels[i].values[2];
#if _DEBUG
            std::cout << "sfm cloud point num " << counter << " : "
                      << "(" << pcl_pnt.x << ", " << pcl_pnt.y << ", " << pcl_pnt.z << "), "
                      << "(" << std::to_string(pcl_pnt.r) << ", " << std::to_string(pcl_pnt.g) << ", " << std::to_string(pcl_pnt.b) << ")"
                      << std::endl;
#endif
            pnt_cloud_temp.push_back(pcl_pnt);
            counter++;
        }
        rv_num ++;
        std::cout << "receive NUM " << rv_num << ":  "<< counter << " sfm pnt cloud points have been updated" << std::endl;
    }

    if(_USE_BUNDINGBOX_FILTER)
    {
        MakeBundingBoxFilter(pnt_cloud_temp);
    }
    for(int i = 0; i < pnt_cloud_temp.size(); i++)
    {
        _sfm_cloud->push_back(pnt_cloud_temp[i]);
    }


    /* receive sfm camera pose data */
    {
        /* for compare */
        cv::Matx31d camera_position;
        camera_position.val[0] = camera_pose_msg->pose.position.x;
        camera_position.val[1] = camera_pose_msg->pose.position.y;
        camera_position.val[2] = -1.0*camera_pose_msg->pose.position.z;
        _camera_positions_4comp.push_back(camera_position);
    }

    /* receive sfm amor pose data */
    {
        /* for compare */
        cv::Matx31d amor_position;
        amor_position.val[0] = -1.0*amor_pose_msg->pose.position.y;
        amor_position.val[1] = -1.0*amor_pose_msg->pose.position.z;
        amor_position.val[2] = amor_pose_msg->pose.position.x;
        _amor_positions_4comp.push_back(amor_position);
    }

    /* update */
    {
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }
}


/* ------------------------------------------------------------------------- */
/* \fn void KeyboardEventOccurred
*
* \brief    receive the keyboard signal, and do corresponding reaction.
*           it includes :
*           key --h-- to show show all possibilities for interaction
*           key --s-- to show or hiden sfm cloud
*           key --l-- to show or hiden laser cloud
*           key --b-- to show or hiden ba cloud
*           key --w-- to show or hiden sfm laser pose
*           key --d-- to show or hiden sfm camera pose
*           key --x-- to open or close compare matx function
*           key --t-- to transform sfm cloud with compare matx
*           key --r-- to go back to (0,0,0,0,-1,0)
*           key --c-- to show or hiden coordinate
*           key --p-- to compare sfm and laser clouds
*           key --m-- to remove all clouds and points
*           key --n-- to show points num in clouds
*           key --KP_1,4-- to make sfm cloud bigger or smaller in x derection
*           key --KP_2,5-- to make sfm cloud bigger or smaller in y derection
*           key --KP_3,6-- to make sfm cloud bigger or smaller in z derection
*           key --a-- to automaticparameterbasedcomparing
*           key --y-- to automaticparameterbasedcomparing with rotation
*
* \param[in]    event --> the keyboard signal
*               viewer_void --> the registered viewer
*/
/* ------------------------------------------------------------------------- */

void VIEWER::
KeyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void *viewer_void)
{
    std::cout << "key board press : " << event.getKeySym() << std::endl;
    if (event.getKeySym() == "r" && event.keyDown())
    {
        std::cout << "reset the camera position to (0, 0, 0, 0, 0, 1, 0, -1, 0)" << std::endl;
        SetCameraPose(pcl::PointXYZ(0,0,0), pcl::PointXYZ(0,0,1), pcl::PointXYZ(0,-1,0));
    }
    if (event.getKeySym() == "c" && event.keyDown())
    {
        if(!show_coordinate)
        {
            std::cout << "show coordinatesystem" << std::endl;
            _viewer->addCoordinateSystem(1.0);
        }
        else
        {
            std::cout << "remove coordinatesystem" << std::endl;
            _viewer->removeCoordinateSystem(0.0);
        }
        show_coordinate = !show_coordinate;
    }
    if (event.getKeySym() == "m" && event.keyDown())
    {
        _sfm_cloud->clear();
        _laser_cloud->clear();
        for(int i = 0; i < 101; i++)
            for(int j = 0; j < 100; j++)
                for(int k = 0; k < 100; k++)
                    (_laser_pnts_box[i][j][k]).clear();
        _camera_positions_4comp.clear();
        _amor_positions_4comp.clear();
        _compare_error = 100000;
        _compare_matx = cv::Matx44d::eye();
        _compare_matx_computet = false;

        _lock.lock();
        _manager_update = true;
        _lock.unlock();
        std::cout << "clear pnt cloud " << std::endl;
    }
    if (event.getKeySym() == "p" && event.keyDown())
    {
        this->DoCompare();
    }


    if (event.getKeySym() == "t" && event.keyDown())
    {
        std::cout << "transform sfm cloud with compare matrix" << std::endl;
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            cv::Matx41d pnt = cv::Matx41d(_sfm_cloud->at(i).x, _sfm_cloud->at(i).y, _sfm_cloud->at(i).z, 1);
            if(transform == false)
                pnt = _compare_matx*pnt;
            else
                pnt = _compare_matx.inv()*pnt;
            _sfm_cloud->at(i).x = pnt.val[0];
            _sfm_cloud->at(i).y = pnt.val[1];
            _sfm_cloud->at(i).z = pnt.val[2];
        }
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
        transform = !transform;
    }
    if (event.getKeySym() == "s" && event.keyDown())
    {
        sfm_cloud_show = !sfm_cloud_show;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }
    if (event.getKeySym() == "l" && event.keyDown())
    {
        laser_cloud_show = !laser_cloud_show;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }
    if (event.getKeySym() == "b" && event.keyDown())
    {
        ba_cloud_show = !ba_cloud_show;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }
    if (event.getKeySym() == "w" && event.keyDown())
    {
        sfm_laser_pose_show = !sfm_laser_pose_show;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }
    if (event.getKeySym() == "d" && event.keyDown())
    {
        sfm_camera_pose_show = !sfm_camera_pose_show;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }
    if (event.getKeySym() == "x" && event.keyDown())
    {
        use_compare_matx = !use_compare_matx;
        if(use_compare_matx)
            std::cout << "use compare max to do comparation" << std::endl;
        else
            std::cout << "do not use compare max to do comparation" << std::endl;
    }

    if (event.getKeySym() == "KP_1" && event.keyDown())
    {
        int x_change_furture = x_change + change_rat;
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            float x_now = _sfm_cloud->at(i).x;
            float x_pre = x_now * 100.0 / ( 100.0 + x_change );
            float x_furture = x_pre*(100.0+x_change_furture) / 100.0;
            _sfm_cloud->at(i).x = x_furture;
        }
        x_change = x_change_furture;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
        std::cout << "change in x derection" << std::endl;
    }

    if (event.getKeySym() == "KP_4" && event.keyDown())
    {
        int x_change_furture = std::max(x_change - change_rat, -90);
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            float x_now = _sfm_cloud->at(i).x;
            float x_pre = x_now * 100.0 / ( 100.0 + x_change );
            float x_furture = x_pre*(100.0+x_change_furture) / 100.0;
            _sfm_cloud->at(i).x = x_furture;
        }
        x_change = x_change_furture;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
        std::cout << "change in x derection" << std::endl;
    }

    if (event.getKeySym() == "KP_2" && event.keyDown())
    {
        int y_change_furture = y_change + change_rat;
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            float y_now = _sfm_cloud->at(i).y;
            float y_pre = y_now * 100.0 / ( 100.0 + y_change );
            float y_furture = y_pre*(100.0+y_change_furture) / 100.0;
            _sfm_cloud->at(i).y = y_furture;
        }
        y_change = y_change_furture;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
        std::cout << "change in y derection" << std::endl;
    }

    if (event.getKeySym() == "KP_5" && event.keyDown())
    {
        int y_change_furture = std::max(y_change - change_rat, -90);;
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            float y_now = _sfm_cloud->at(i).y;
            float y_pre = y_now * 100.0 / ( 100.0 + y_change );
            float y_furture = y_pre*(100.0+y_change_furture) / 100.0;
            _sfm_cloud->at(i).y = y_furture;
        }
        y_change = y_change_furture;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
        std::cout << "change in y derection" << std::endl;
    }


    if (event.getKeySym() == "KP_3" && event.keyDown())
    {
        int z_change_furture = z_change + change_rat;
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            float z_now = _sfm_cloud->at(i).z;
            float z_pre = z_now * 100.0 / ( 100.0 + z_change );
            float z_furture = z_pre*(100.0+z_change_furture) / 100.0;
            _sfm_cloud->at(i).z = z_furture;
        }
        z_change = z_change_furture;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
        std::cout << "change in z derection" << std::endl;
    }

    if (event.getKeySym() == "KP_6" && event.keyDown())
    {
        int z_change_furture = std::max(z_change - change_rat, -90);;
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            float z_now = _sfm_cloud->at(i).z;
            float z_pre = z_now * 100.0 / ( 100.0 + z_change );
            float z_furture = z_pre*(100.0+z_change_furture) / 100.0;
            _sfm_cloud->at(i).z = z_furture;
        }
        z_change = z_change_furture;
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
        std::cout << "change in z derection" << std::endl;
    }
    if (event.getKeySym() == "n" && event.keyDown())
    {
        std::cout << "num of laser_cloud : " << _laser_cloud->size() << std::endl;
        std::cout << "num of sfm_cloud : " << _sfm_cloud->size() << std::endl;

        float x_max = -1000000, x_min = 1000000;
        float y_max = -1000000, y_min = 1000000;
        float z_max = -1000000, z_min = 1000000;

        int laser_size = _laser_cloud->size();
        for(int i = 0; i < laser_size; i++)
        {
            if(i%50 != 0)
                continue;
            x_max = x_max < _laser_cloud->at(i).x ? _laser_cloud->at(i).x : x_max;
            x_min = x_min > _laser_cloud->at(i).x ? _laser_cloud->at(i).x : x_min;

            y_max = y_max < _laser_cloud->at(i).y ? _laser_cloud->at(i).y : y_max;
            y_min = y_min > _laser_cloud->at(i).y ? _laser_cloud->at(i).y : y_min;

            z_max = z_max < _laser_cloud->at(i).z ? _laser_cloud->at(i).z : z_max;
            z_min = z_min > _laser_cloud->at(i).z ? _laser_cloud->at(i).z : z_min;
        }
        int use_num = 0, unu_num = 0;
        for(int i = 0; i < _laser_pnts_box.size(); i++)
            for(int j = 0; j < _laser_pnts_box[i].size(); j++)
                for(int k = 0; k < _laser_pnts_box[i][j].size(); k++)
                {
                    if(_laser_pnts_box[i][j][k].empty())
                        unu_num++;
                    else
                        use_num++;
                }

        std::cout << " laser point cloud size :"
                  << "x : ( " <<  x_min << ", " << x_max << " ),  "
                  << "y : ( " <<  y_min << ", " << y_max << " ),  "
                  << "z : ( " <<  z_min << ", " << z_max << " ),  "
                  << "used block : " << use_num << ", "
                  << "use rate : " << float(use_num)/((x_max-x_min)*(y_max-y_min)*(z_max-z_min)) << std::endl;
    }
    if (event.getKeySym() == "a" && event.keyDown())
    {
        this->AutomaticParameterBasedCompare();
    }
    if (event.getKeySym() == "y" && event.keyDown())
    {
        this->AutomaticParameterBasedAngleCompare();
    }

    if (event.getKeySym() == "KP_7" && event.keyDown())
    {
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            _sfm_cloud->at(i).x = _sfm_cloud->at(i).x+0.01;
        }
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }

    if (event.getKeySym() == "Tab" && event.keyDown())
    {
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            _sfm_cloud->at(i).x = _sfm_cloud->at(i).x-0.01;
        }
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }

    if (event.getKeySym() == "KP_8" && event.keyDown())
    {
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            _sfm_cloud->at(i).y = _sfm_cloud->at(i).y+0.01;
        }
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }

    if (event.getKeySym() == "KP_Divide" && event.keyDown())
    {
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            _sfm_cloud->at(i).y = _sfm_cloud->at(i).y-0.01;
        }
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }

    if (event.getKeySym() == "KP_9" && event.keyDown())
    {
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            _sfm_cloud->at(i).z = _sfm_cloud->at(i).z+0.01;
        }
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }

    if (event.getKeySym() == "KP_Multiply" && event.keyDown())
    {
        for(int i = 0; i < _sfm_cloud->size(); i++)
        {
            _sfm_cloud->at(i).z = _sfm_cloud->at(i).z-0.01;
        }
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }

    if (event.getKeySym() == "h" && event.keyDown())
    {
        std::cout << "key --s-- to show or hiden sfm cloud" << std::endl;
        std::cout << "key --l-- to show or hiden laser cloud" << std::endl;
        std::cout << "key --b-- to show or hiden ba cloud" << std::endl;
        std::cout << "key --w-- to show or hiden sfm laser pose" << std::endl;
        std::cout << "key --d-- to show or hiden sfm camera pose" << std::endl;
        std::cout << "key --x-- to open or close compare matx function" << std::endl;
        std::cout << "key --t-- to transform sfm cloud with compare matx" << std::endl;
        std::cout << "key --r-- to go back to (0,0,0,0,-1,0)" << std::endl;
        std::cout << "key --c-- to show or hiden coordinate" << std::endl;
        std::cout << "key --p-- to compare sfm and laser clouds" << std::endl;
        std::cout << "key --m-- to remove all clouds and points" << std::endl;
        std::cout << "key --n-- to show points num in clouds" << std::endl;
        std::cout << "key --KP_1,4-- to make sfm cloud bigger or smaller in x derection" << std::endl;
        std::cout << "key --KP_2,5-- to make sfm cloud bigger or smaller in y derection" << std::endl;
        std::cout << "key --KP_3,6-- to make sfm cloud bigger or smaller in z derection" << std::endl;
        std::cout << "key --a-- to automaticparameterbasedcomparing" << std::endl;
        std::cout << "key --y-- to automaticparameterbasedcomparing with rotation" << std::endl;
    }

}

/* ------------------------------------------------------------------------- */
/* \fn bool DoCompare
*
* \brief checking the suitability between laser cloud and sfm cloud
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool VIEWER::DoCompare()
{
    double time = cv::getTickCount();
    std::cout << "doing compare between laser and sfm cloud" << std::endl;
    // compute new compare matx
    {
        if(use_compare_matx)
        {
            std::cout << "computing compare matx..." << std::endl;
            if(!ComputeCompareMatx())
            {
                cout << "error in VIEWER::DoCompare(), ComputeCompareMatx() return false" << std::endl;
                return false;
            }
            std::cout << "compute compare matrix is done !" << std::endl
                      << "compare matx error is " << _compare_error<< std::endl
                      << _compare_matx << std::endl;
            cv::Mat X;
            cv::eigen(_compare_matx, X);
            std::cout << "X : " << X << std::endl;
        }
    }
    std::cout << "making... compare between sfm and laser clouds" << std::endl;
    if(use_compare_matx)
        std::cout << "compare clouds with +++++ compare matrix" << std::endl;
    else
        std::cout << "compare clouds without -----compare matrix" << std::endl;

    int pnts_meet = 0, pnts_miss = 0;
    {
        int sfm_cloud_size = _sfm_cloud->size();
        for(int n = 0; n < sfm_cloud_size; n++)
        {
            pcl::PointXYZRGB pcl_pnt = _sfm_cloud->at(n);
            cv::Point3d check_pnt;
            if(use_compare_matx)
            {
                cv::Matx41d pnt_matx;
                pnt_matx.val[0] = pcl_pnt.x;
                pnt_matx.val[1] = pcl_pnt.y;
                pnt_matx.val[2] = pcl_pnt.z;
                pnt_matx.val[3] = 1;
                pnt_matx = _compare_matx*pnt_matx;
                check_pnt = cv::Point3d(pnt_matx.val[0], pnt_matx.val[1], pnt_matx.val[2]);
            }
            else
            {
                check_pnt = cv::Point3d(pcl_pnt.x, pcl_pnt.y, pcl_pnt.z);
            }

            /* check the positon of laser point in box */
            {
                int x_meet = 0, y_meet = 0, z_meet = 0;
#if USE_COMPARE_BLOCK
                if(check_pnt.x+50 >= 0 && check_pnt.x+50 < 100 &&
                        check_pnt.y+50 >= 0 && check_pnt.y+50 < 100 &&
                        check_pnt.z+50 >= 0 && check_pnt.z+50 < 100 )
                {
                    x_meet = int(check_pnt.x+50);
                    y_meet = int(check_pnt.y+50);
                    z_meet = int(check_pnt.z+50);
                }
                else
                {
                    x_meet = 100;
                    y_meet = 99;
                    z_meet = 99;
                }
#endif
                if(CheckMeetOrNot(check_pnt, _laser_pnts_box[x_meet][y_meet][z_meet], _points_distance))
                    pnts_meet++;
                else
                    pnts_miss++;
            }
        }
    }
    std::cout << "cloud compare is done!" <<
                 " pnts_meet is " << pnts_meet << " ; " <<
                 " pnts_miss is " << pnts_miss << " ; " <<
                 " pnts meet rat is " << float(pnts_meet)/float(pnts_meet+pnts_miss) << " ; ";

    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    std::cout << "manuell compare is done. take (" << time << "s). "
#if USE_COMPARE_BLOCK
              << "with compare blocks"
#else
              << "without compare blocks"
#endif
              << std::endl;

    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn bool CheckMeetOrNot
*
* \brief checking the suitability between one point in sfm cloud and the points in laser cloud
*
* \param[in]    sfm_cloud_point --> target pnt in sfm cloud
*               laser_pnts_box --> the corresponding points box for laser cloud
*               error --> the distance between points.
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool VIEWER::CheckMeetOrNot(const cv::Point3d& sfm_cloud_pnt, const std::vector<int>& laser_pnts_box, const float& error)
{
#if USE_COMPARE_BLOCK
    if(laser_pnts_box.empty())
        return false;
    int box_size = laser_pnts_box.size();
    for(int i = 0; i < box_size; i++)
    {
        cv::Point3d laser_cloud_pnt((_laser_cloud->at(laser_pnts_box[i])).x, (_laser_cloud->at(laser_pnts_box[i])).y, (_laser_cloud->at(laser_pnts_box[i])).z);
        if(cv::norm(laser_cloud_pnt-sfm_cloud_pnt) < error)
            return true;
    }
#else
    int laser_size = _laser_cloud->size();
    if(laser_size <= 0)
        return false;
    for(int i = 0; i < laser_size; i++)
    {
        cv::Point3d laser_pnt((_laser_cloud->at(i)).x, (_laser_cloud->at(i)).y, (_laser_cloud->at(i)).z);
        if(cv::norm(laser_pnt-sfm_cloud_point) < error)
            return true;
    }
#endif
    return false;
}

/* ------------------------------------------------------------------------- */
/* \fn bool ComputeCompareMatx
*
* \brief    compute the compare matrix C, with C*Ps = Pl. C is a mapping matrix from sfm-cloud to laser-cloud.
*           and store it in _compare_matx
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool VIEWER::ComputeCompareMatx()
{
    int camera_positions_num = _camera_positions_4comp.size();
    int amor_positions_num = _amor_positions_4comp.size();

    std::cout << "camera_positions_num is " << camera_positions_num << std::endl
              << "amor_positions_num is " << amor_positions_num << std::endl;

    if(amor_positions_num > camera_positions_num)
    {
        std::cout << "error in function VIEWER::ComputeCompareMatx(), camera_positions_num > amor_positions_num" << std::endl;
        return false;
    }
    int min_num = 3;
    int max_num = 9;
    if(amor_positions_num < min_num)
    {
        std::cout << "error in function VIEWER::ComputeCompareMatx(), please wait for the camera position num > " << min_num <<  std::endl;
        return false;
    }

    std::vector<cv::Matx31d> pa, pc;
    for(int i = 0; i < std::max(amor_positions_num, max_num); i++)
    {
        pa.push_back(_amor_positions_4comp[i]);
        pc.push_back(_camera_positions_4comp[i]);
    }
    // Matx*PC = PA  Matx = PA*PC.inv()
    std::vector<cv::Matx31d> pc_extra, pa_extra;    const cv::Matx31d V(0,-1,0);
    std::vector<cv::Matx44d> compare_matx_all;
    for(int i = 0; i < std::max(amor_positions_num, max_num); i++)
    {
        for(int j = i+1; j < std::max(amor_positions_num, max_num); j++)
        {
            for(int k = j+1; k < std::max(amor_positions_num, max_num); k++)
            {
                cv::Matx31d pc_e = 0.33333*(pc[i]+pc[j]+pc[k]) +
                        (cv::norm(pc[i]-pc[j])+cv::norm(pc[i]-pc[k])+cv::norm(pc[j]-pc[k]))*V;
                pc_extra.push_back(pc_e);
                cv::Matx44d PC = cv::Matx44d(pc[i].val[0], pc[j].val[0], pc[k].val[0], pc_e.val[0],
                        pc[i].val[1], pc[j].val[1], pc[k].val[1], pc_e.val[1],
                        pc[i].val[2], pc[j].val[2], pc[k].val[2], pc_e.val[2],
                        1, 1, 1, 1);

                cv::Matx31d pa_e = 0.33333*(pa[i]+pa[j]+pa[k]) +
                        (cv::norm(pa[i]-pa[j])+cv::norm(pa[i]-pa[k])+cv::norm(pa[j]-pa[k]))*V;
                pa_extra.push_back(pa_e);
                cv::Matx44d PA = cv::Matx44d(pa[i].val[0], pa[j].val[0], pa[k].val[0], pa_e.val[0],
                        pa[i].val[1], pa[j].val[1], pa[k].val[1], pa_e.val[1],
                        pa[i].val[2], pa[j].val[2], pa[k].val[2], pa_e.val[2],
                        1, 1, 1, 1);
                cv::Matx44d PCinv;
                cv::invert(PC, PCinv, cv::DECOMP_SVD);
                cv::Matx44d Matx = PA*PCinv;

                compare_matx_all.push_back(Matx);
            }
        }
    }
    float min_error = 100000.0;    cv::Matx44d min_error_matx;
    {
        bool done = false;
        for(int c_i = 0; c_i < compare_matx_all.size(); c_i++)
        {
            cv::Matx14d check_matx = cv::Matx14d(0,0,0,1)*compare_matx_all[c_i];
            if(cv::norm(check_matx) > 1+10e-6)
                continue;
            float local_error = 0;
            for(int p_i = 0; p_i < pc.size(); p_i++)
            {
                cv::Matx41d pci = compare_matx_all[c_i]*cv::Matx41d(pc[p_i].val[0], pc[p_i].val[1], pc[p_i].val[2], 1);
                local_error += cv::norm(pci - cv::Matx41d(pa[p_i].val[0], pa[p_i].val[1], pa[p_i].val[2], 1));
            }
            for(int e_i = 0; e_i < pc_extra.size(); e_i++)
            {
                cv::Matx41d pcei = compare_matx_all[c_i]*
                        cv::Matx41d(pc_extra[e_i].val[0], pc_extra[e_i].val[1], pc_extra[e_i].val[2], 1);
                local_error +=
                        cv::norm(pcei - cv::Matx41d(pa_extra[e_i].val[0], pa_extra[e_i].val[1], pa_extra[e_i].val[2], 1));
            }
            local_error = local_error / float(pc.size()+pc_extra.size());
            if(min_error > local_error)
            {
                min_error = local_error;
                min_error_matx = compare_matx_all[c_i];
                done = true;
            }
        }
        if(!done)
            return false;
    }
    _compare_matx = min_error_matx;
    _compare_error = min_error;
    _compare_matx_computet = true;
    return true;
}

/* ------------------------------------------------------------------------- */
/* \fn void SetCameraPose
*
* \brief    set the camera postion
*
* \param[in]    position --> camera position
*               direction --> camera view derection
*               up --> up vector for camera
*/
/* ------------------------------------------------------------------------- */
void VIEWER::
SetCameraPose(const pcl::PointXYZ position, const pcl::PointXYZ direction, const pcl::PointXYZ up)
{
    _viewer->setCameraPosition(position.x, position.y, position.z,
                               direction.x, direction.y, direction.z,
                               up.x, up.y, up.z);
}

/* ------------------------------------------------------------------------- */
/* \fn void MakeBundingBoxFilter
*
* \brief    make bunding box filter for a poindcloud
*
* \param[in]    point_cloud --> target point cloud
*
* \return if seccessful, return true. otherwise return false
*/
/* ------------------------------------------------------------------------- */
bool VIEWER::
MakeBundingBoxFilter(std::vector<pcl::PointXYZRGB>& point_cloud)
{
    std::vector<int> bbox_filter(point_cloud.size(), 1);
    for(int i = 0; i < point_cloud.size(); i++)
    {
        pcl::PointXYZRGB pnt_i = point_cloud[i];
        for(int ii = _sfm_cloud->size()-1; ii >= 0 && _sfm_cloud->size()-ii <= 6000; ii--)
        {
            pcl::PointXYZRGB pnt_ii = _sfm_cloud->at(ii);
            float norm = sqrt((pnt_i.x-pnt_ii.x)*(pnt_i.x-pnt_ii.x)+
                              (pnt_i.y-pnt_ii.y)*(pnt_i.y-pnt_ii.y)+
                              (pnt_i.z-pnt_ii.z)*(pnt_i.z-pnt_ii.z));
            if(norm < _bbox_radius)
            {
                bbox_filter[i] = 0;
                break;
            }
        }
    }

    std::vector<pcl::PointXYZRGB> point_cloud_temp;
    int count = 0;
    for(int i = 0; i < bbox_filter.size(); ++i)
    {
        if(bbox_filter[i] == 1)
            point_cloud_temp.push_back(point_cloud[i]);
        else
            count ++;
    }
    point_cloud = point_cloud_temp;

    std::cout << count << " points have been filter " << std::endl;
}



/* ------------------------------------------------------------------------- */
/* \fn void laserSyncCallback
*
* \brief    synchronized receive msgs from ros laser node, then store them in arrays
*
* \param[in]    pose_msg --> amor pose msg
*               cloud_msg --> laser point cloud msg
*/
/* ------------------------------------------------------------------------- */
int receiver_count = 0;
void VIEWER::
laserSyncCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg, const sensor_msgs::PointCloudConstPtr &cloud_msg)
{
    receiver_count ++;
    if(receiver_count % 5 != 0)
        return;

    /* for die amor pose */
    float qw = pose_msg->pose.orientation.w;
    float qx = pose_msg->pose.orientation.x;
    float qy = pose_msg->pose.orientation.y;
    float qz = pose_msg->pose.orientation.z;
    float t0 = pose_msg->pose.position.x;
    float t1 = pose_msg->pose.position.y;
    float t2 = pose_msg->pose.position.z;

    float sqw = qw*qw; float sqx = qx*qx; float sqy = qy*qy; float sqz = qz*qz;
    float invs = 1.0/(sqw + sqx + sqy + sqz);
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

    cv::Matx33d amor_rotation(m00, m01, m02,
                              m10, m11, m12,
                              m20, m21, m22);
    cv::Matx31d amor_transform(-1.0*t1, -1.0*t2, t0);

    /* for point cloud */
    std::vector<pcl::PointXYZRGB> laser_cloud;
    {
        for(int i = 0; i < cloud_msg->points.size(); i++)
        {
            if(i % 10 != 0)
                continue;
            cv::Matx31d pnt_cv;
            pnt_cv.val[0] = cloud_msg->points[i].x;
            pnt_cv.val[1] = cloud_msg->points[i].y;
            pnt_cv.val[2] = cloud_msg->points[i].z;

            pnt_cv = amor_rotation*pnt_cv;

            pcl::PointXYZRGB pnt_pcl;
            pnt_pcl.x = -1.0*pnt_cv.val[0] + amor_transform.val[0];
            pnt_pcl.y = -1.0*pnt_cv.val[2] + amor_transform.val[1];
            pnt_pcl.z = -1.0*pnt_cv.val[1] + amor_transform.val[2];

            pnt_pcl.r = 200;
            pnt_pcl.g = 200;
            pnt_pcl.b = 200;

            laser_cloud.push_back(pnt_pcl);
        }
        pcl::PointXYZRGB pose_pnt;
        pose_pnt.x = -1*t1;
        pose_pnt.y = -1*t2;
        pose_pnt.z = t0;
        pose_pnt.r = 0;
        pose_pnt.g = 0;
        pose_pnt.b = 255;
        laser_cloud.push_back(pose_pnt);
    }
    /* store in laser_cloud */
    {
        for(int n = 0; n < laser_cloud.size(); n++)
        {
#if USE_COMPARE_BLOCK
            /* full compare box */
            if(laser_cloud[n].x+50 >= 0 && laser_cloud[n].x+50 < 100 &&
                    laser_cloud[n].y+50 >= 0 && laser_cloud[n].y+50 < 100 &&
                    laser_cloud[n].z+50 >= 0 && laser_cloud[n].z+50 < 100 )
            {
                (_laser_pnts_box[int(laser_cloud[n].x+50)][int(laser_cloud[n].y+50)][int(laser_cloud[n].z+50)]).push_back(_laser_cloud->size());
            }
            else
            {
                (_laser_pnts_box[100][99][99]).push_back(_laser_cloud->size());
            }
#endif
            /* store in laser cloud */
            _laser_cloud->push_back(laser_cloud[n]);
        }
        //        std::cout << laser_cloud.size()<<" laser points have been updatet" << std::endl;
    }

    /* update */
    {
        _lock.lock();
        _manager_update = true;
        _lock.unlock();
    }
}
/* ------------------------------------------------------------------------- */
/* \fn bool AutomaticParameterBasedCompare
*
* \brief    make auto compare in x, y and z derections.
*           and then set the sfm-cloud in the best pose
*/
/* ------------------------------------------------------------------------- */
bool VIEWER::AutomaticParameterBasedCompare()
{
    double time = cv::getTickCount();

    unsigned int laser_size = _laser_cloud->size();
    if(laser_size == 0)
    {
        std::cout << "can not do auto compare, laser size = 0" << std::endl;
        return false;
    }

    int x_pose_min = -1, x_pose_max = 1;
    int y_pose_min = -1, y_pose_max = 1;
    int z_pose_min = -1, z_pose_max = 1;
    int dx_p = 1, dy_p = 1, dz_p = 1;
    int pose_div = 20;

    int x_size_min = -1, x_size_max = 1;
    int y_size_min = -1, y_size_max = 1;
    int z_size_min = -1, z_size_max = 1;
    int dx_s = 1, dy_s = 1, dz_s = 1;
    int size_div = 50;

    float meet_rat = 0.0;
    int x_size_store = 0, y_size_store = 0, z_size_store = 0;
    int x_pose_store = 0, y_pose_store = 0, z_pose_store = 0;

    std::vector<pcl::PointXYZRGB> sfm_cloud;
    unsigned int sfm_size = _sfm_cloud->size();
    for(int num = 0; num < sfm_size; num++)
    {
        sfm_cloud.push_back(_sfm_cloud->at(num));
    }


    if(sfm_size == 0)
        return false;

    std::cout << " preparation complete " << std::endl;
    for(int x_pose = x_pose_min; x_pose <= x_pose_max; x_pose = x_pose+dx_p)
    {
        for(int y_pose = y_pose_min; y_pose <= y_pose_max; y_pose = y_pose+dy_p)
        {
            for(int z_pose = z_pose_min; z_pose <= z_pose_max; z_pose = z_pose+dz_p)
            {
                for(int x_size = x_size_min; x_size <= x_size_max; x_size = x_size+dx_s)
                {
                    for(int y_size = y_size_min; y_size <= y_size_max; y_size = y_size+dy_s)
                    {
                        for(int z_size = z_size_min; z_size <= z_size_max; z_size = z_size+dz_s)
                        {
#if DEBUG
                            std::cout << "pose:( " << x_pose << "," << y_pose << "," << z_pose << "), "
                                      << "size:( " << x_size << "," << y_size << "," << z_size << "), "
                                      << "meet rat: " << meet_rat << std::endl;
#endif
                            int pnts_meet = 0, pnts_miss = 0;
                            for(int i = 0; i < sfm_size; i++)
                            {
                                pcl::PointXYZRGB pnt = sfm_cloud[i];
                                pnt.x = pnt.x * (1+float(x_size)/float(size_div));
                                pnt.x = pnt.x + float(x_pose)/float(pose_div);
                                pnt.y = pnt.y * (1+float(y_size)/float(size_div));
                                pnt.y = pnt.y + float(y_pose)/float(pose_div);
                                pnt.z = pnt.z * (1+float(z_size)/float(size_div));
                                pnt.z = pnt.z + float(z_pose)/float(pose_div);

                                _sfm_cloud->at(i) = pnt;

                                if(i%(std::max(int(sfm_size*0.005), 10)) != 0)
                                    continue;
                                cv::Point3d check_pnt(pnt.x, pnt.y, pnt.z);
                                {
                                    int x_meet = 0, y_meet = 0, z_meet = 0;
                                    if(check_pnt.x+50 >= 0 && check_pnt.x+50 < 100 &&
                                            check_pnt.y+50 >= 0 && check_pnt.y+50 < 100 &&
                                            check_pnt.z+50 >= 0 && check_pnt.z+50 < 100 )
                                    {
                                        x_meet = int(check_pnt.x+50);
                                        y_meet = int(check_pnt.y+50);
                                        z_meet = int(check_pnt.z+50);
                                    }
                                    else
                                    {
                                        x_meet = 100;
                                        y_meet = 99;
                                        z_meet = 99;
                                    }
                                    if(CheckMeetOrNot(check_pnt, _laser_pnts_box[x_meet][y_meet][z_meet], _points_distance))
                                        pnts_meet++;
                                    else
                                        pnts_miss++;
                                }
                            }
                            float local_meet_rat = float(pnts_meet)/float(pnts_meet+pnts_miss);
                            if(meet_rat < local_meet_rat)
                            {
                                meet_rat = local_meet_rat;
                                x_size_store = x_size;  y_size_store = y_size;  z_size_store = z_size;
                                x_pose_store = x_pose;  y_pose_store = y_pose;  z_pose_store = z_pose;
                            }
                            // update
                            _all_cloud->clear();
                            for(int i = 0; i < sfm_size && sfm_cloud_show; i++)
                                _all_cloud->push_back(_sfm_cloud->at(i));
                            _viewer->updatePointCloud(_all_cloud, _cloudName);
                            _viewer->spinOnce(10);

                        }
                    }
                }
            }
        }
    }
    std::cout << "AutomaticParameterBasedComparing done ! meet rat : " << meet_rat << std::endl
              << "x, y, z pose and div : " << x_pose_store << ", " << y_pose_store << ", " << z_pose_store << ", " << pose_div << std::endl
              << "x, y, z size and div : " << x_size_store << ", " << y_size_store << ", " << z_size_store << ", " << size_div << std::endl;
    for(int i = 0; i < _sfm_cloud->size(); i++)
    {
        pcl::PointXYZRGB pnt = sfm_cloud[i];
        pnt.x = pnt.x * (1+float(x_size_store)/float(size_div));
        pnt.x = pnt.x + float(x_pose_store)/float(pose_div);
        pnt.y = pnt.y * (1+float(y_size_store)/float(size_div));
        pnt.y = pnt.y + float(y_pose_store)/float(pose_div);
        pnt.z = pnt.z * (1+float(z_size_store)/float(size_div));
        pnt.z = pnt.z + float(z_pose_store)/float(pose_div);
        _sfm_cloud->at(i) = pnt;

    }
    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    std::cout << "auto compare is done. take (" << time << "s). "
#if USE_COMPARE_BLOCK
              << "with compare blocks"
#else
              << "without compare blocks"
#endif
              << std::endl;
    _lock.lock();
    _manager_update = true;
    _lock.unlock();
    return true;
}
/* ------------------------------------------------------------------------- */
/* \fn bool AutomaticParameterBasedAngleCompare
*
* \brief    make auto compare with rotations in x, y and z derections.
*           and then set the sfm-cloud in the best pose
*/
/* ------------------------------------------------------------------------- */

bool VIEWER::AutomaticParameterBasedAngleCompare()
{
    double time = cv::getTickCount();

    unsigned int laser_size = _laser_cloud->size();
    if(laser_size == 0)
    {
        std::cout << "can not do auto compare, laser size = 0" << std::endl;
        return false;
    }

    int x_rot_min = -1, x_rot_max = 1;
    int y_rot_min = -1, y_rot_max = 1;
    int z_rot_min = -1, z_rot_max = 1;
    int dx_r = 1, dy_r = 1, dz_r = 1;
    float rot_agl = PI/180.0;

    float meet_rat = 0.0;
    int x_rot_store = 0, y_rot_store = 0, z_rot_store = 0;

    std::vector<pcl::PointXYZRGB> sfm_cloud;
    unsigned int sfm_size = _sfm_cloud->size();
    for(int num = 0; num < sfm_size; num++)
    {
        sfm_cloud.push_back(_sfm_cloud->at(num));
    }
    if(sfm_size == 0)
        return false;
    for(int x_rot = x_rot_min; x_rot <= x_rot_max; x_rot = x_rot+dx_r)
    {
        for(int y_rot = y_rot_min; y_rot <= y_rot_max; y_rot = y_rot+dy_r)
        {
            for(int z_rot = z_rot_min; z_rot <= z_rot_max; z_rot = z_rot+dz_r)
            {
                int pnts_meet = 0, pnts_miss = 0;
                for(int i = 0; i < sfm_size; i++)
                {
                    pcl::PointXYZRGB pnt = sfm_cloud[i];

                    cv::Matx33d Rx(1, 0, 0,
                                   0, std::cos(x_rot*rot_agl), -1*std::sin(x_rot*rot_agl),
                                   0, std::sin(x_rot*rot_agl), std::cos(x_rot*rot_agl));

                    cv::Matx33d Ry(std::cos(y_rot*rot_agl), 0, std::sin(y_rot*rot_agl),
                                   0, 1, 0,
                                   -1*std::sin(y_rot*rot_agl), 0, std::cos(y_rot*rot_agl));

                    cv::Matx33d Rz(std::cos(z_rot*rot_agl), -1*std::sin(z_rot*rot_agl), 0,
                                   std::sin(z_rot*rot_agl), std::cos(z_rot*rot_agl), 0,
                                   0, 0, 1);

                    cv::Matx31d pnt_cv(pnt.x, pnt.y, pnt.z);
                    pnt_cv = Rz*Ry*Rx*pnt_cv;
                    pnt.x = pnt_cv.val[0], pnt.y = pnt_cv.val[1], pnt.z = pnt_cv.val[2];
                    _sfm_cloud->at(i) = pnt;

                    if(i%(std::max(int(sfm_size*0.005), 10)) != 0)
                        continue;
                    cv::Point3d check_pnt(pnt.x, pnt.y, pnt.z);
                    {
                        int x_meet = 0, y_meet = 0, z_meet = 0;
                        if(check_pnt.x+50 >= 0 && check_pnt.x+50 < 100 &&
                                check_pnt.y+50 >= 0 && check_pnt.y+50 < 100 &&
                                check_pnt.z+50 >= 0 && check_pnt.z+50 < 100 )
                        {
                            x_meet = int(check_pnt.x+50);
                            y_meet = int(check_pnt.y+50);
                            z_meet = int(check_pnt.z+50);
                        }
                        else
                        {
                            x_meet = 100;
                            y_meet = 99;
                            z_meet = 99;
                        }
                        if(CheckMeetOrNot(check_pnt, _laser_pnts_box[x_meet][y_meet][z_meet], _points_distance))
                            pnts_meet++;
                        else
                            pnts_miss++;
                    }
                }
                float local_meet_rat = float(pnts_meet)/float(pnts_meet+pnts_miss);
                if(meet_rat < local_meet_rat)
                {
                    meet_rat = local_meet_rat;
                    x_rot_store = x_rot;    y_rot_store = y_rot;    z_rot_store = z_rot;
                }
                // update
                _all_cloud->clear();
                for(int i = 0; i < sfm_size && sfm_cloud_show; i++)
                    _all_cloud->push_back(_sfm_cloud->at(i));
                _viewer->updatePointCloud(_all_cloud, _cloudName);
                _viewer->spinOnce(10);

            }
        }
    }
    std::cout << "AutomaticParameterBasedComparing done ! meet rat : " << meet_rat << std::endl
              << "x, y, z rot and agl :  " << x_rot_store << "," << y_rot_store << "," << z_rot_store << ", " << rot_agl << std::endl;
    for(int i = 0; i < _sfm_cloud->size(); i++)
    {
        pcl::PointXYZRGB pnt = sfm_cloud[i];

        cv::Matx33d Rx(1, 0, 0,
                       0, std::cos(x_rot_store*rot_agl), -1*std::sin(x_rot_store*rot_agl),
                       0, std::sin(x_rot_store*rot_agl), std::cos(x_rot_store*rot_agl));

        cv::Matx33d Ry(std::cos(y_rot_store*rot_agl), 0, std::sin(y_rot_store*rot_agl),
                       0, 1, 0,
                       -1*std::sin(y_rot_store*rot_agl), 0, std::cos(y_rot_store*rot_agl));

        cv::Matx33d Rz(std::cos(z_rot_store*rot_agl), -1*std::sin(z_rot_store*rot_agl), 0,
                       std::sin(z_rot_store*rot_agl), std::cos(z_rot_store*rot_agl), 0,
                       0, 0, 1);

        cv::Matx31d pnt_cv(pnt.x, pnt.y, pnt.z);
        pnt_cv = Rz*Ry*Rx*pnt_cv;
        pnt.x = pnt_cv.val[0], pnt.y = pnt_cv.val[1], pnt.z = pnt_cv.val[2];
        _sfm_cloud->at(i) = pnt;

    }

    time = ((double)cv::getTickCount() - time)/cv::getTickFrequency();
    std::cout << "auto rotation compare is done. take (" << time << "s). "
#if USE_COMPARE_BLOCK
              << "with compare blocks"
#else
              << "without compare blocks"
#endif
              << std::endl;


    _lock.lock();
    _manager_update = true;
    _lock.unlock();
    return true;

}

/* ------------------------------------------------------------------------- */
/* \fn void BACallback
*
* \brief    receive the ba point cloud or for other point cloud. and store it.
*
* \param[in]  cloud_msg --> target cloud message
*/
/* ------------------------------------------------------------------------- */
void VIEWER::
BACallback(const sensor_msgs::PointCloudConstPtr& cloud_msg)
{
#if ONLY_FOR_BA
    _ba_cloud->clear();
#endif
    for(int i = 0; i < cloud_msg->points.size(); i++)
    {
        pcl::PointXYZRGB pcl_pnt;
        pcl_pnt.x = cloud_msg->points[i].x;
        pcl_pnt.y = cloud_msg->points[i].y;
        pcl_pnt.z = cloud_msg->points[i].z;

        pcl_pnt.r = cloud_msg->channels[i].values[0];
        pcl_pnt.g = cloud_msg->channels[i].values[1];
        pcl_pnt.b = cloud_msg->channels[i].values[2];
        _ba_cloud->push_back(pcl_pnt);
    }
    std::cout <<
#if ONLY_FOR_BA
                 "BA points have been update : "
#else
                 "3D Points have been update : "
#endif
              << cloud_msg->points.size() << std::endl;

    _lock.lock();
    _manager_update = true;
    _lock.unlock();

}
