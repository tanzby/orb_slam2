/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "orb_slam2_core/System.h"

using namespace std;

class ImageGrabber {
public:
    ImageGrabber(orb_slam2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabImage(const sensor_msgs::ImageConstPtr &msg);

    orb_slam2::System *mpSLAM;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "orb_mono_node");
    ros::NodeHandle nh("~");

    std::string vocabulary_path;
    nh.param<std::string>("vocabulary_path", vocabulary_path, "");
    if (vocabulary_path.empty()) {
        ROS_ERROR("vocabulary_path cannot be empty");
        nh.shutdown();
        exit(-1);
    }

    std::string settings_path;
    nh.param<std::string>("settings_path", settings_path, "");
    if (settings_path.empty()) {
        ROS_ERROR("settings_path cannot be empty");
        nh.shutdown();
        exit(-1);
    }

    ROS_WARN("vocabulary_path: %s", vocabulary_path.c_str());
    ROS_WARN("settings_path: %s", settings_path.c_str());

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    orb_slam2::System SLAM(vocabulary_path, settings_path, orb_slam2::System::MONOCULAR, true);

    ImageGrabber igb(&SLAM);

    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    /*
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    */

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
}


