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
#include <iostream>
#include <thread>
#include <ros/ros.h>

int main(int argc, char **argv) {
    std::cout << "hello world " << std::endl;

    ros::init(argc, argv, "camera_process", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    ImageProcess imageProcess(nh);

    std::thread process_thread(&ImageProcess::Process, &imageProcess);

    ros::spin();

    process_thread.join();

    return 1;
}
