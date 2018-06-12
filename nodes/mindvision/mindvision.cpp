/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
  This program requires ROS to be installed
  Author: 	David Wong (david.wong@tier4.jp)
						Mingyang Ma (mamingyang129@gmail.com)
  Initial version 		2018-05-21
*/

#include "CameraApi.h"

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <signal.h>

#define DISPLAY_MAX 2

using namespace cv;
using namespace std;

unsigned char* g_pRgbBuffer;     // Data buffer

static volatile int running = 1;

static void signal_handler(int)
{
	running = 0;
	ros::shutdown();
}

void parse_camera_info(const cv::Mat& camMat,
                       const cv::Mat& disCoeff,
                       const cv::Size& imgSize,
                       sensor_msgs::CameraInfo& msg)
{
	msg.header.frame_id = "camera";
	msg.height = imgSize.height;
	msg.width  = imgSize.width;

	for (int row=0; row<3; row++)
	{
		for (int col=0; col<3; col++)
		{
			msg.K[row * 3 + col] = camMat.at<double>(row, col);
		}
	}

	for (int row=0; row<3; row++)
	{
		for (int col=0; col<4; col++)
		{
			if (col == 3)
			{
				msg.P[row * 4 + col] = 0.0f;
			} else
			{
				msg.P[row * 4 + col] = camMat.at<double>(row, col);
			}
		}
	}

	for (int row=0; row<disCoeff.rows; row++)
	{
		for (int col=0; col<disCoeff.cols; col++)
		{
			msg.D.push_back(disCoeff.at<double>(row, col));
		}
	}
}



/*!
 * Reads and parses the Autoware calibration file format
 * @param nh ros node handle
 * @param camerainfo_msg CameraInfo message to fill
 */
void getMatricesFromFile(const ros::NodeHandle& nh,
													sensor_msgs::CameraInfo &camerainfo_msg)
{

	cv::Mat  cameraExtrinsicMat;
	cv::Mat  cameraMat;
	cv::Mat  distCoeff;
	cv::Size imageSize;
	std::string filename;

	if (nh.getParam("calibrationfile", filename) && filename!="")
	{
		ROS_INFO("Trying to parse calibrationfile :");
		ROS_INFO("> %s", filename.c_str());
	}
	else
	{
		ROS_INFO("No calibrationfile param was received");
		return;
	}

	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		ROS_INFO("Cannot open %s", filename.c_str());;
		return;
	}
	else
	{
		fs["CameraMat"] >> cameraMat;
		fs["DistCoeff"] >> distCoeff;
		fs["ImageSize"] >> imageSize;
	}
	parse_camera_info(cameraMat, distCoeff, imageSize, camerainfo_msg);
}

/*!
 * Reads the params from the console
 * @param private_nh[in] Private Ros node handle
 * @param fps[out] Read value from the console double
 * @param mode[out] Read value from the console integer
 * @param format[out] Read value from the console mono or rgb
 * @param timeout[out] Read value from the console timeout in ms
 */
void ros_get_params(const ros::NodeHandle& private_nh,
										int& fps,
										int& mode,
										std::string& format,
										int& timeout)
{
	if (private_nh.getParam("fps", fps))
	{
		ROS_INFO("fps set to %d", fps);
	} else {
		fps = 20;
		ROS_INFO("No param received, defaulting fps to %d", fps);
	}

	if (private_nh.getParam("mode", mode))
	{
		ROS_INFO("mode set to %d", mode);
	} else {
		mode = 0;
		ROS_INFO("No param received, defaulting mode to %d", mode);
	}

	if (private_nh.getParam("format", format))
	{
		ROS_INFO("format set to %s", format.c_str());
	} else {
		format = "rgb";
		ROS_INFO("No param received, defaulting format to %s", format.c_str());
	}

	if (private_nh.getParam("timeout", timeout))
	{
		ROS_INFO("timeout set to %d ms", timeout);
	} else {
		timeout = 1000;
		ROS_INFO("No param received, defaulting timeout to %d ms", timeout);
	}
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "mindvision");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	signal(SIGTERM, signal_handler); //detect closing
	int fps, camera_mode, timeout;
	std::string format;
	ros_get_params(private_nh, fps, camera_mode, format, timeout);

	// int iCameraCounts = 2;
	int iCameraCounts = 2;
	int iStatus=-1;
	tSdkCameraDevInfo tCameraEnumList[iCameraCounts];
	int hCamera[DISPLAY_MAX] = {-1,-1};
	tSdkCameraCapbility tCapability[iCameraCounts];      // Device info
	tSdkFrameHead           sFrameInfo;
	BYTE*	pbyBuffer;
	//int iDisplayFrames = 10000;
	cv::Mat image;
	int channel = 3;

	CameraSdkInit(1);
	// Build device list
	int status_id = CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);
	ROS_INFO("the return id is %d", status_id);
  	// Check for connected devices
  	if(iCameraCounts==0){
    	std::cout << "Couldn't find MindVision camera" << std::endl;
    	return -1;
	}

	for(int i=0; i<iCameraCounts; i++)
	{
		iStatus = CameraInit(&tCameraEnumList[i],-1,-1,&hCamera[i]);
		if (iStatus!=CAMERA_STATUS_SUCCESS)
		{
			std::cout << "Couldn't initialize MindVision camera" << i << std::endl;
			std::cout << "Status: " << iStatus << std::endl;
			continue;
		}
		CameraGetCapability(hCamera[i],&tCapability[i]);
		g_pRgbBuffer = (unsigned char*)malloc(tCapability[i].sResolutionRange.iHeightMax*tCapability[i].sResolutionRange.iWidthMax*3);
		CameraPlay(hCamera[i]);
		if(tCapability[i].sIspCapacity.bMonoSensor){
			CameraSetIspOutFormat(hCamera[i],CAMERA_MEDIA_TYPE_MONO8);
		}
		else{
			CameraSetIspOutFormat(hCamera[i],CAMERA_MEDIA_TYPE_RGB8);
		}
	}

	// calibration data
	sensor_msgs::CameraInfo camerainfo_msg;
	getMatricesFromFile(private_nh, camerainfo_msg);

	ros::Publisher publishers_cameras[iCameraCounts];
	ros::Publisher camera_info_pub[iCameraCounts];
	ros::NodeHandle node_handle;

	for (int i=0; i< iCameraCounts; i++)
	{
		std::string current_topic = "camera" + std::to_string(i) + "/image_raw";
		publishers_cameras[i] = node_handle.advertise<sensor_msgs::Image>(current_topic, 100);
	}

	// TO DO:
	// camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1, true);
	//
	//
	// std::string topic(std::string("image_raw"));
	//pub = n.advertise<sensor_msgs::Image>(topic, 100);\
	ROS_INFO("Publishing...");


	std::cout << "Capturing with " << iCameraCounts << " MindVision cameras" << std::endl;

	int count = 0;
	ros::Rate loop_rate(fps); // Hz


  while (running && ros::ok())
  {
		for (int i=0; i< iCameraCounts; i++)
		{
			if(CameraGetImageBuffer(hCamera[i],&sFrameInfo,&pbyBuffer,timeout) == CAMERA_STATUS_SUCCESS)
			{
				CameraImageProcess(hCamera[i], pbyBuffer, g_pRgbBuffer,&sFrameInfo);
				cv::Mat image;
				image = cv::Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3, g_pRgbBuffer, cv::Mat::AUTO_STEP);
				cv::resize(image,image,cv::Size(1024,1024));
				CameraReleaseImageBuffer(hCamera[i],pbyBuffer);
				sensor_msgs::ImagePtr msg;
				std_msgs::Header header;
				msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg();
				msg->header.frame_id = "camera";
				msg->header.stamp.sec = ros::Time::now().sec;
				msg->header.stamp.nsec = ros::Time::now().nsec;
				msg->header.seq = count;
				publishers_cameras[i].publish(msg);
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}

	//close cameras
	for (int i=0; i< iCameraCounts; i++)
	{
		CameraUnInit(hCamera[i]);
	}

  free(g_pRgbBuffer);

	ROS_INFO("Camera node closed correctly");
	return 0;
}
