/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of FL2SAM (https://github.com/JokerJohn/FL2SAM-GPS).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Xiangcheng Hu (xhubd@connect.ust.hk.com)
 * Date: ${Date}
 * Description: 
 *******************************************************/
#ifndef SRC_IMAGE_PROCESS_H
#define SRC_IMAGE_PROCESS_H

#include <fstream>
#include <chrono>
#include<thread>
#include<queue>
#include<mutex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>


#include <eigen3/Eigen/Dense>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace Eigen;

class ImageProcess {

public:
    ImageProcess(ros::NodeHandle &nh);

    ~ImageProcess();

    /**
     * main loop
     */
    void Process();

private:

    /**
     * call back function for camera 0
     * @param msg
     */
    void CameraCallback0(const sensor_msgs::ImageConstPtr &msg);

    /**
     * set camera extrinsincs and extrinsincs
     * @param type
     */
    void SetCameraIntrincs(const int type);

    /**
     * distort images
     * @param mat
     * @param type
     */
    void ImageDistort(cv::Mat &mat, int type);

    ros::Subscriber subCamera0Images;
    image_transport::Subscriber subImage_0, subImage_1;
    std::queue<sensor_msgs::ImageConstPtr> camera0Buf;

    std::mutex mutexLock;

    tf::TransformListener m_tfListener;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;  // 畸变系数

    // extrinsanc
    Eigen::Matrix3d k_matrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Identity();

    std::ofstream outputx, outputy;
    std::string saveDirectory;

    int index = 0;
};


#endif //SRC_IMAGE_PROCESS_H
