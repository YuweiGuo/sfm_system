#ifndef SETTING_H
#define SETTING_H

#include <opencv2/opencv.hpp>

/* camera setting */
const cv::Matx33d camera_matx(1000.0,  0.0000,   640,   // camera matrix
                              0.0000,  1000.0,   360,
                              0.0000,  0.0000,   1.0);
const cv::Matx41d dist_coeff(0, 0, 0, 0);   // dist matrix
const int camera_fps = 60;  // frame per second for cameras


/* ros node setting */
#define AMOR_IMG_NODE_NAME "/amor/frontCam/image"
#define AMOR_LASER_POINT_CLOUD_NODE_NAME "/amor/scan"
#define AMOR_POSE_NODE_NAME "/amor/pose"
#define SFM_MANAGER_NODE_NAME "sfm_manager"
#define SFM_VIEWER_NODE_NAME "sfm_viwer"
#define SFM_BA_NODE_NAME "sfm_ba"
#define SFM_MANAGER_CLOUD_NAME "/sfm_manager/pointcloud"
#define SFM_MANAGER_CLOUD4SHOW_NAME "/sfm_manager/pointcloud4show"
#define SFM_MANAGER_CAMERA_POSE_NAME "/sfm_manager/camera_pose"
#define SFM_MANAGER_AMOR_POSE_NAME "/sfm_manager/amor_pose"
#define SFM_MANAGER_BA_REQUEST_NAME "/sfm_manager/SfM_BA_Request_Msg"
#define SFM_VIEWER_BA_REQUEST_NAME "/sfm_viewer/SfM_BA_Request_Msg"
#define SFM_BA_CLOUD_NAME "/sfm_ba/SfM_BA_Cloud_Msg"

#define OTHER_IMAGE_NODE "/amor/frontCam/image"

#define USE_FASTORB_METHOD 1    // if true,  use FASTORB_METHOD for the feature matching , otherwise, use the default methode SIFT
#define FOR_IMG_AND_POSE 1      // if true,  use the image and poses receiver and operations for sfm-process, otherwise, only use image receiver and operations

#endif // SETTING_H
