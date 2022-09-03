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
#include "../include/image_process.h"


ImageProcess::ImageProcess(ros::NodeHandle &nh) {
    // load params
    nh.param<std::string>("save_directory", saveDirectory, "~/Download/fl2sam");
    if (saveDirectory.back() != '/') {
        saveDirectory.append("/");
    }
    outputx.open(saveDirectory + "realsense-corners_x.txt", std::ios::out);
    outputy.open(saveDirectory + "realsense-corners_y.txt", std::ios::out);

    image_transport::ImageTransport it(nh);
    subCamera0Images = nh.subscribe("/camera/color/image_raw", 10, &ImageProcess::CameraCallback0, this);
}

ImageProcess::~ImageProcess() {
    outputx.close();
    outputy.close();
}

void ImageProcess::Process() {
    while (1) {
        while (!camera0Buf.empty()) {
            mutexLock.lock();
            sensor_msgs::ImageConstPtr camera0_msg = camera0Buf.front();
            camera0Buf.pop();
            mutexLock.unlock();

            bool imageReceived = true;
            cv_bridge::CvImagePtr cv_left_ptr;
            try {
                cv_left_ptr = cv_bridge::toCvCopy(*camera0_msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception) {
                imageReceived = false;
                std::cout << "cv_bridge ERROR" << std::endl;
            }

            bool transformReceived = true;
            ros::Time align_time = ros::Time().fromSec(camera0_msg->header.stamp.toSec() - 4);
            m_tfListener.waitForTransform("/base", "/tool0_controller", align_time, ros::Duration(1.0));
            tf::StampedTransform sensorToBaseTf;
            try {
                m_tfListener.lookupTransform("/base", "tool0_controller", align_time, sensorToBaseTf);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
                transformReceived = false;
            }

            if (!imageReceived || !transformReceived){
                ROS_ERROR("WAITING FOR PROPER TF OR IMAGES");
                continue;
            }

            ROS_INFO("rotation: %f, %f, %f, %f", sensorToBaseTf.getRotation().x(), sensorToBaseTf.getRotation().y(),
                     sensorToBaseTf.getRotation().z(), sensorToBaseTf.getRotation().w());

            index++;
            cv::Mat image_mat = cv_left_ptr->image;
            // segmment fault
            // ImageDistort(image_mat, 0);

            Mat gray;
            cvtColor(image_mat, gray, cv::COLOR_BGR2GRAY);
            waitKey(1);

            std::vector<cv::Point2f> corners;
            bool found = findChessboardCorners(gray, cv::Size(8, 11), corners, cv::CALIB_CB_ADAPTIVE_THRESH);
            if (found) {
                cornerSubPix(gray, corners, Size(8, 11), Size(-1, -1),
                             TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                outputx << index << "\t";
                outputy << index << "\t";
                for (int j = 0; j < 88; ++j) {
                    outputx << corners[j].x << "\t";
                    outputy << corners[j].y << "\t";
                }
                outputx << std::endl;
                outputy << std::endl;
            }

            drawChessboardCorners(gray, cv::Size(8, 11), Mat(corners), found);

            std::cout << corners << std::endl;
            imshow("result2", gray);
            cv::imshow("teset imasge", image_mat);
            cv::waitKey(20);
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void ImageProcess::CameraCallback0(const sensor_msgs::ImageConstPtr &msg) {
    mutexLock.lock();
    camera0Buf.push(msg);
    mutexLock.unlock();
}


void ImageProcess::SetCameraIntrincs(const int type) {

//    height: 480
//    width: 640
//    distortion_model: "plumb_bob"
//    D: [0.0, 0.0, 0.0, 0.0, 0.0]
//    K: [615.937255859375, 0.0, 325.6908874511719, 0.0, 616.138916015625, 239.04701232910156, 0.0, 0.0, 1.0]
//    R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
//    P: [615.937255859375, 0.0, 325.6908874511719, 0.0, 0.0, 616.138916015625, 239.04701232910156, 0.0, 0.0, 0.0, 1.0, 0.0]
//    binning_x: 0
//    binning_y: 0
//    roi:
//    x_offset: 0
//    y_offset: 0
//    height: 0
//    width: 0
//    do_rectify: False

    if (type == 0) {
        cameraMatrix.at<double>(0, 0) = 615.937255859375;
        cameraMatrix.at<double>(0, 2) = 325.6908874511719;
        cameraMatrix.at<double>(1, 1) = 616.138916015625;
        cameraMatrix.at<double>(1, 2) = 239.04701232910156;
        cameraMatrix.at<double>(2, 2) = 1;

        distCoeffs.at<double>(0, 0) = -0.;
        distCoeffs.at<double>(1, 0) = -0.;
        distCoeffs.at<double>(2, 0) = 0.;
        distCoeffs.at<double>(3, 0) = -0.;
    } else
        std::cout << "NO CAMERA TYPE" << std::endl;

}


void ImageProcess::ImageDistort(cv::Mat &mat, int type) {
    cv::Mat image = mat;
    cv::Mat frame;
    cv::Mat imageCalibration = cv::Mat::zeros(mat.rows, mat.cols, CV_64F);

    SetCameraIntrincs(type);

    cv::Mat view, rview, map1, map2;
    cv::Size imageSize;
    imageSize = image.size();
    // 图像去畸变，计算两个映射关系
    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                         getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize,
                                                                   0),
                                         imageSize, CV_16SC2, map1, map2);

    remap(image, imageCalibration, map1, map2, cv::INTER_LINEAR);


    // normal camera
    //  cv::fisheye::undistortPoints(
    //      RawCoordinates, RectCoordinates, cvKmat, cvDistCoeff, cvRectMat, cvPmat);
    //cv::Size sensor_size(width_, height_);
    //  cv::Mat cvPmat(3, 4, CV_64F);
    //  cv::eigen2cv(P_, cvPmat);
    //  cv::fisheye::undistortPoints(
    //      RawCoordinates, RectCoordinates, cvKmat, cvDistCoeff, cvRectMat, cvPmat);
    //  cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs,  cv::Mat(), cvPmat,
    //                                       imageSize, CV_32FC1, map1, map2);
    //  cv::Mat cvSrcMask = cvSrcMask.ones(imageSize.height, imageSize.width, CV_32F);
    //  cv::Mat cvDstMask = cvSrcMask.zeros(imageSize.height, imageSize.width, CV_32F);
    //  cv::remap(cvSrcMask, cvDstMask, map1, map2, cv::INTER_LINEAR);
    //  cv::threshold(cvDstMask, cvDstMask, 0.1, 255, cv::THRESH_BINARY);
    //  cvDstMask.convertTo(cvDstMask, CV_8U);
    //cv::cv2eigen(cvDstMask, UndistortRectify_mask_);

    mat = imageCalibration;
}
