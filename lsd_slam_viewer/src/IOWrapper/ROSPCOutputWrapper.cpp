/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ROSPCOutputWrapper.h"
#include <ros/ros.h>

// Publish Additional Data
#include "image_transport/image_transport.h"

#include <opencv2/opencv.hpp>
// For sleeping.
#include <unistd.h>

ROSPCOutputWrapper::ROSPCOutputWrapper(int width, int height)
{
	this->width = width;
	this->height = height;
  image_transport::ImageTransport it(nh_);
	//pc_image_channel_name = nh_.resolveName("lsd_slam/pc_image");
  pc_image_publisher = it.advertise("lsd_slam/pc_image", 1);
	//pc_image_publisher = nh_.advertise<image_transport::ImageTransport>(pc_image_channel_name,1);
  printf("ROSPCOutputWrapper. Should be initialized.\n");

}

ROSPCOutputWrapper::~ROSPCOutputWrapper()
{
}
void ROSPCOutputWrapper::publishPointCloudImage(cv::Mat* img_matrix){
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *img_matrix).toImageMsg();
    pc_image_publisher.publish(msg);
}
