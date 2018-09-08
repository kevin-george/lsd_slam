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

#pragma once

#include <ros/ros.h>

//class image_transport;
#include "image_transport/image_transport.h"

#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


/** Addition to LiveSLAMWrapper for ROS interoperability. */
class ROSPCOutputWrapper
{
	public:

		// initializes cam-calib independent stuff
		ROSPCOutputWrapper(int width, int height);
		~ROSPCOutputWrapper();

		virtual void publishPointCloudImage(cv::Mat* img_matrix);
	//	virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data);



	private:
		int width, height;

    std::string pc_image_channel_name;
		image_transport::Publisher pc_image_publisher;
		ros::NodeHandle nh_;

    sensor_msgs::ImagePtr msg;
};
