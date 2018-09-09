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
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/


#include "KeyFrameGraphDisplay.h"
#include "KeyFrameDisplay.h"
#include "settings.h"
#include <sstream>
#include <fstream>

// For debugging timing.
#include <chrono>

#include "ros/package.h"

#include "IOWrapper/ROSPCOutputWrapper.h"

#include <opencv2/opencv.hpp>

using namespace std::chrono;

KeyFrameGraphDisplay::KeyFrameGraphDisplay()
{
	flushPointcloud = false;
	printNumbers = false;
        // Initialize the outputWrapper
}

KeyFrameGraphDisplay::~KeyFrameGraphDisplay()
{
	for(unsigned int i=0;i<keyframes.size();i++)
		delete keyframes[i];
}



void KeyFrameGraphDisplay::publishPointCloudImage(GLubyte **pixels) {
    size_t i, j, k, cur;
    const size_t format_nchannels = 3;

    *pixels = (GLubyte*)realloc(*pixels, format_nchannels * sizeof(GLubyte) * width * height);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, *pixels);
		//GLenum error;

		if (glGetError() != GL_NO_ERROR){
			printf("Error while publishing. Skipping.\n");
			return;
		}

    cv::Mat temp_matrix(height,width, CV_8UC3, cv::Scalar(0,0,0));
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            cur = format_nchannels * ((height - i - 1) * width + j);
				    temp_matrix.at<cv::Vec3b>(i,j)[0] = (*pixels)[cur];
				    temp_matrix.at<cv::Vec3b>(i,j)[1] = (*pixels)[cur+1];
				    temp_matrix.at<cv::Vec3b>(i,j)[2] = (*pixels)[cur+2];

        }

    }


    if (outputWrapper != nullptr)
			 outputWrapper->publishPointCloudImage(&temp_matrix);
}

void KeyFrameGraphDisplay::setOutputWrapper(ROSPCOutputWrapper* outputWrapper){
     this->outputWrapper = outputWrapper;
}


void KeyFrameGraphDisplay::draw()
{
	dataMutex.lock();
	numRefreshedAlready = 0;

	// draw keyframes
	float color[3] = {0,0,1};
	for(unsigned int i=0;i<keyframes.size();i++)
	{
		if(showKFCameras)
			keyframes[i]->drawCam(lineTesselation, color);

		if((showKFPointclouds && (int)i > cutFirstNKf) || i == keyframes.size()-1)
			keyframes[i]->drawPC(pointTesselation, 1);
	}
  if (publish_counter % publish_frequency == 0){

      publishPointCloudImage(&pixels);
      if (publish_counter > 300)
         publish_counter = 0;
  }
  publish_counter ++;

	if(flushPointcloud)
	{

		printf("Flushing Pointcloud to %s!\n", (ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		std::ofstream f((ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		int numpts = 0;
		for(unsigned int i=0;i<keyframes.size();i++)
		{
			if((int)i > cutFirstNKf)
				numpts += keyframes[i]->flushPC(&f);
		}
		f.flush();
		f.close();

		std::ofstream f2((ros::package::getPath("lsd_slam_viewer")+"/pc.pcd").c_str());
		f2 << std::string("# .PCD v0.7 - Point Cloud Data file format\n");
		f2 << std::string("VERSION 0.7\n");
		f2 << std::string("FIELDS x y z rgb\n");
		f2 << std::string("SIZE 4 4 4 4\n");
		f2 << std::string("TYPE F F F F\n");
		f2 << std::string("COUNT 1 1 1 1\n");
		f2 << std::string("WIDTH ") << numpts << std::string("\n");
		f2 << std::string("HEIGHT 1\n");
		f2 << std::string("VIEWPOINT 0 0 0 1 0 0 0\n");
		f2 << std::string("POINTS ") << numpts << std::string("\n");
		f2 << std::string("DATA binary\n");

		std::ifstream f3((ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		while(!f3.eof()) f2.put(f3.get());

		f2.close();
		f3.close();

		system(("rm "+ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		flushPointcloud = false;
		printf("Done Flushing Pointcloud with %d points!\n", numpts);

	}


	if(printNumbers)
	{
		int totalPoint = 0;
		int visPoints = 0;
		for(unsigned int i=0;i<keyframes.size();i++)
		{
			totalPoint += keyframes[i]->totalPoints;
			visPoints += keyframes[i]->displayedPoints;
		}

		printf("Have %d points, %d keyframes, %d constraints. Displaying %d points.\n",
				totalPoint, (int)keyframes.size(), (int)constraints.size(), visPoints);
		printNumbers = false;
	}




	if(showConstraints)
	{
		// draw constraints
		glLineWidth(lineTesselation);
		glBegin(GL_LINES);
		for(unsigned int i=0;i<constraints.size();i++)
		{
			if(constraints[i].from == 0 || constraints[i].to == 0)
				continue;

			double colorScalar = std::max(0.0, std::min(1.0, constraints[i].err / 0.05));
			glColor3f(colorScalar, 1 - colorScalar, 0);


			Sophus::Vector3f t = constraints[i].from->camToWorld.translation();
			glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);

			t = constraints[i].to->camToWorld.translation();
			glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);

		}
		glEnd();
	}

	dataMutex.unlock();
}

void KeyFrameGraphDisplay::addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	dataMutex.lock();
	if(keyframesByID.count(msg->id) == 0)
	{
		KeyFrameDisplay* disp = new KeyFrameDisplay();
		keyframesByID[msg->id] = disp;
		keyframes.push_back(disp);

	//	printf("added new KF, now there are %d!\n", (int)keyframes.size());
	}

	keyframesByID[msg->id]->setFrom(msg);
	dataMutex.unlock();
}

void KeyFrameGraphDisplay::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	dataMutex.lock();

	constraints.resize(msg->numConstraints);
	assert(msg->constraintsData.size() == sizeof(GraphConstraint)*msg->numConstraints);
	GraphConstraint* constraintsIn = (GraphConstraint*)msg->constraintsData.data();
	for(int i=0;i<msg->numConstraints;i++)
	{
		constraints[i].err = constraintsIn[i].err;
		constraints[i].from = 0;
		constraints[i].to = 0;

		if(keyframesByID.count(constraintsIn[i].from) != 0)
			constraints[i].from = keyframesByID[constraintsIn[i].from];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].from);


		if(keyframesByID.count(constraintsIn[i].to) != 0)
			constraints[i].to = keyframesByID[constraintsIn[i].to];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].to);
	}



	GraphFramePose* graphPoses = (GraphFramePose*)msg->frameData.data();
	int numGraphPoses = msg->numFrames;
	assert(msg->frameData.size() == sizeof(GraphFramePose)*msg->numFrames);

	for(int i=0;i<numGraphPoses;i++)
	{
		if(keyframesByID.count(graphPoses[i].id) == 0)
		{
		//	printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[i].id, graphPoses[i].id);
		}
		else
			memcpy(keyframesByID[graphPoses[i].id]->camToWorld.data(), graphPoses[i].camToWorld, 7*sizeof(float));
	}

	dataMutex.unlock();

//	printf("graph update: %d constraints, %d poses\n", msg->numConstraints, msg->numFrames);
}
