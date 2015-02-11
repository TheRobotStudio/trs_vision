/*
 * Copyright (c) 2014, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Sep 3, 2014
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int32MultiArray.h>
#include <rosgraph_msgs/Log.h>
#include <trs_msgs/MotorDataSet.h>
#include <std_msgs/String.h>
#include <cv.h> //test
#include <vector>
#include <cvaux.h>
#include <cxcore.hpp>
#include <sys/stat.h>
#include <termios.h>
#include <term.h>
#include <unistd.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE 						50
#define MAIN_SCREEN_WIDTH 				1600
#define LEGS_CURRENT_DISPLAY_OFFSET		800
#define SLAVE_LEFT_ARM_ID		1
#define SLAVE_RIGHT_ARM_ID		2
#define SLAVE_HEAD_ID			3
#define SLAVE_LEFT_LEG_ID		4
#define SLAVE_RIGHT_LEG_ID		5
#define NUMBER_MAX_EPOS2_PER_SLAVE 		15

/*** Namespaces ***/
using namespace std;
using namespace cv;

/*** Class ***/
class CvSurfacePro2GUI
{
public:

	CvSurfacePro2GUI(std::string name) :
    it_(nh_)
	{
		//TV display
		cvNamedWindow("Maxon Motor - The Robot Studio - TV GUI", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Maxon Motor - The Robot Studio - TV GUI", MAIN_SCREEN_WIDTH-1, -27);
		cvSetWindowProperty("Maxon Motor - The Robot Studio - TV GUI", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

		//Surface pro 2 display
		cvNamedWindow("Maxon Motor - The Robot Studio", CV_WINDOW_AUTOSIZE); 	// output screen
		cvMoveWindow("Maxon Motor - The Robot Studio", 0, -27);
		cvSetWindowProperty("Maxon Motor - The Robot Studio", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

		cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 1.0, 4.0, 2,2,CV_AA);
		textColor = CV_RGB(0,255,255);

		//Subscribers
		sub_motorDataSet = nh_.subscribe("/motorDataSet", 1, &CvSurfacePro2GUI::motorDataSet_cb, this);
		sub_rosout = nh_.subscribe("/rosout", 1, &CvSurfacePro2GUI::rosout_cb, this);

		//Image transport Subscribers and Publishers
		sub_imgEyeBall = it_.subscribe("/image/imgEyeBall", 1, &CvSurfacePro2GUI::imgEyeBall_cb, this);
		sub_imgDetectedHead = it_.subscribe("/image/imgDetectedHead", 1, &CvSurfacePro2GUI::imgDetectedHead_cb, this);
		sub_structurePixyMerged = it_.subscribe("/image/structurePixyMerged", 1, &CvSurfacePro2GUI::structurePixyMerged_cb, this);

		backgroundImg = cv::imread(ros::package::getPath("trs_vision") + "/resources/gui_TV_v8.jpg");
		surfaceBckgndImg = cv::imread(ros::package::getPath("trs_vision") + "/resources/gui_SP2_v1.jpg");

		/*
		double alpha = 0.1; double beta;
		beta = ( 1.0 - alpha );
		cv::addWeighted(src1, alpha, scr2, beta, 0.0, dst);
		*/

		imgEyeBall_received = false;
		imgDetectedHead_received = false;
		structurePixyMerged_received = false;
		motorDataSet_arrived = false;
		rosout_arrived = false;

		for(int i=0; i<75; i++)
		{
			prev_motorDataSet.motorData[i].encPosition = 0;
			encResetCnt[i] = -1;
		}
	}

	~CvSurfacePro2GUI(void)
	{
		cvDestroyWindow("Maxon Motor - The Robot Studio - TV GUI");
		cvDestroyWindow("Maxon Motor - The Robot Studio");
	}

	/*** Callback methods ***/
	void imgEyeBall_cb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr1;

		//convert from ros image format to opencv image format
		try
		{
		  cv_ptr1 = cv_bridge::toCvCopy(msg, "bgr8");
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		//cvReleaseImageHeader(&imgEyeBall_ptr);

		imgEyeBall_received = true;

		//create a temp img mat
		cv::Mat finalView = backgroundImg.clone();

		if(imgEyeBall_received)
		{
			//add camera view
			cv::Mat src1 = cv_ptr1->image;//imgEyeBall_ptr;
			src1.copyTo(finalView(Rect(52, 150, src1.cols, src1.rows)));
		}

		//Draw current lines
		if(motorDataSet_arrived)
		{
			//arms
			for(int i=0; i<12; i++)
			{
				Scalar color = Scalar(140, 130, 40);

				int left_current = 0;
				stringstream ss;

				if((i>=5)&&(i<=8)) 
				{
					ss << encResetCnt[(SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+(i+2)];
					left_current = motorDataSet.motorData[(SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+(i+2)].current;
					if(motorDataSet.motorData[(SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+(i+2)].encPosition < 0) color = Scalar(0, 0, 255);
				}
				else if((i>=9)&&(i<=14)) 
				{
					ss << encResetCnt[(SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+(i+3)];
					left_current = motorDataSet.motorData[(SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+(i+3)].current;
					if(motorDataSet.motorData[(SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+(i+3)].encPosition < 0) color = Scalar(0, 0, 255);
				}
				else
				{
					ss << encResetCnt[(SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i];
					left_current = motorDataSet.motorData[(SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i].current;
					if(motorDataSet.motorData[(SLAVE_LEFT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i].encPosition < 0) color = Scalar(0, 0, 255);
				}

				//clip
				if(left_current>1000) left_current=1000;
				//drawn left to right for left arm
				cv::line(finalView, Point(580, 777+i*21), Point(580+(left_current/10), 777+i*21), color, 5, 4);

				//int zero = 1;

				//ss << encResetCnt[i];
				//string resetEncCnt_str = std::(zero);//.c_str()
				//resetEncCnt_str << "0";

				cv::putText(finalView, ss.str(), cv::Point(685,777+i*21), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(140, 130, 40), 1, 8, false);
			}


			for(int i=0; i<12; i++)
			{
				Scalar color = Scalar(140, 130, 40);
				stringstream ss;

				//motorDataSet
				int right_current = motorDataSet.motorData[(SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i].current;
				//clip
				if(right_current>1000) right_current=1000;

				if(motorDataSet.motorData[(SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i].encPosition < 0) color = Scalar(0, 0, 255);

				ss << encResetCnt[(SLAVE_RIGHT_ARM_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i];
				//drawn right to left for right arm
				cv::line(finalView, Point(540, 777+i*21), Point(540-(right_current/10), 777+i*21), color, 5, 4);

				cv::putText(finalView, ss.str(), cv::Point(420,777+i*21), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(140, 130, 40), 1, 8, false);
			}

			//legs
			for(int i=0; i<14; i++)
			{


				//motorDataSet
				int left_current = motorDataSet.motorData[(SLAVE_LEFT_LEG_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i].current;
				int right_current = motorDataSet.motorData[(SLAVE_RIGHT_LEG_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i].current;
				//clip
				if(left_current>1000) left_current=1000;
				if(right_current>1000) right_current=1000;

				Scalar colorLeft = Scalar(140, left_current/4, 40);
				Scalar colorRight = Scalar(140, right_current/4, 40);

				if(motorDataSet.motorData[(SLAVE_LEFT_LEG_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i].encPosition < 0) colorLeft = Scalar(0, 0, 255);
				if(motorDataSet.motorData[(SLAVE_RIGHT_LEG_ID-1)*NUMBER_MAX_EPOS2_PER_SLAVE+i].encPosition < 0) colorRight = Scalar(0, 0, 255);

				//drawn right to left for right arm
				cv::line(finalView, Point(540+LEGS_CURRENT_DISPLAY_OFFSET, 770+i*19), Point(540-(right_current/10)+LEGS_CURRENT_DISPLAY_OFFSET, 770+i*19), colorRight, 5, 4);
				//drawn left to right for left arm
				cv::line(finalView, Point(580+LEGS_CURRENT_DISPLAY_OFFSET, 770+i*19), Point(580+(left_current/10)+LEGS_CURRENT_DISPLAY_OFFSET, 770+i*19), colorLeft, 5, 4);
			}
		}

		//Draw console text
		if(rosout_arrived)
		{
			std_msgs::String str;
			str.data = rosoutLog.msg;
			//ROS_INFO("%s", str.data.c_str());

			cv::putText(finalView, str.data.c_str(),cv::Point(1600,777), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(140, 130, 40), 1, 8, false);
		}

		//show image
		cv::imshow("Maxon Motor - The Robot Studio - TV GUI", finalView);
		cvWaitKey(1);
		cv::imshow("Maxon Motor - The Robot Studio", surfaceBckgndImg);
		cvWaitKey(1);
		finalView.release();

		imgEyeBall_received = false;
		//imgDetectedHead_received = false;
		structurePixyMerged_received = false;
		motorDataSet_arrived = false;
		rosout_arrived = false;

		//cvReleaseImage(&imgEyeBall_ptr);
		//cvReleaseImage(&imgDetectedHead_ptr);
		//cvReleaseImage(&structurePixyMerged_ptr);
	}

	void imgDetectedHead_cb(const sensor_msgs::ImageConstPtr& msg)
	{
		//imgDetectedHead_ptr = msg;

		cv_bridge::CvImagePtr cv_ptr;
		//convert from ros image format to opencv image format
		try
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		//imgDetectedHead_received = true;

		//IplImage imgDetectedHead = cv_ptr->image;
		//imgDetectedHead_ptr = cvCloneImage(&imgDetectedHead);
		//imgDetectedHead_ptr = resizeImage(imgDetectedHead_ptr, 256, 256);


		//add face
		resize(cv_ptr->image,cv_ptr->image,cvSize(256,256));
		//cv::Mat src2 = resizeImage(&imgDetectedHead, 256, 256);// imgDetectedHead_ptr2;
		//src2.copyTo(backgroundImg(Rect(96, 750, src2.cols, src2.rows)));
		cv_ptr->image.copyTo(backgroundImg(Rect(60, 760, cv_ptr->image.cols, cv_ptr->image.rows)));

		//cvReleaseImage();
		//cvReleaseImage(imgDetectedHead);
	}

	//display picture here
	void structurePixyMerged_cb(const sensor_msgs::ImageConstPtr& msg)
	{
		//structurePixyMerged_ptr = msg;

		cv_bridge::CvImagePtr cv_ptr;
		//convert from ros image format to opencv image format
		try
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		cv_ptr->image.copyTo(backgroundImg(Rect(1222, 150, cv_ptr->image.cols, cv_ptr->image.rows)));
		cv_ptr->image.copyTo(surfaceBckgndImg(Rect(210, 867, cv_ptr->image.cols, cv_ptr->image.rows)));
		/*

		structurePixyMerged_received = true;

		structurePixyMerged = cv_ptr->image;
		structurePixyMerged_ptr = cvCloneImage(&structurePixyMerged);*/
	}

	void motorDataSet_cb(const trs_msgs::MotorDataSetConstPtr& data)
	{
		motorDataSet = *data;
		motorDataSet_arrived = true;

		//check for encoder reset
		for(int i=0; i<75; i++)
		{
			if(abs(prev_motorDataSet.motorData[i].encPosition - motorDataSet.motorData[i].encPosition) >= 10000)
				encResetCnt[i]++;
		}

		prev_motorDataSet = *data;
	}

	void rosout_cb(const rosgraph_msgs::LogConstPtr& log)
	{
		rosoutLog = *log;
		rosout_arrived = true;
	}

	/*** Methods ***/
	IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight)
	{
		IplImage *outImg = 0;
		int origWidth;
		int origHeight;
		if (origImg) {
			origWidth = origImg->width;
			origHeight = origImg->height;
		}
		if (newWidth <= 0 || newHeight <= 0 || origImg == 0 || origWidth <= 0 || origHeight <= 0) {
			ROS_INFO("ERROR in resizeImage: Bad desired image size of %dx%d.", newWidth, newHeight);
			exit(1);
		}

		// Scale the image to the new dimensions, even if the aspect ratio will be changed.
		outImg = cvCreateImage(cvSize(newWidth, newHeight), origImg->depth, origImg->nChannels);
		if (newWidth > origImg->width && newHeight > origImg->height) {
			// Make the image larger
			cvResetImageROI((IplImage*)origImg);
			cvResize(origImg, outImg, CV_INTER_LINEAR);	// CV_INTER_CUBIC or CV_INTER_LINEAR is good for enlarging
		}
		else {
			// Make the image smaller
			cvResetImageROI((IplImage*)origImg);
			cvResize(origImg, outImg, CV_INTER_AREA);	// CV_INTER_AREA is good for shrinking / decimation, but bad at enlarging.
		}

		return outImg;
	}

protected:

	ros::NodeHandle nh_;
	CvFont font;
    CvScalar textColor;
    cv::Mat backgroundImg;
    cv::Mat surfaceBckgndImg;
	image_transport::ImageTransport it_;
	image_transport::Subscriber sub_imgEyeBall;
	image_transport::Subscriber sub_imgDetectedHead;
	image_transport::Subscriber sub_structurePixyMerged;
	ros::Subscriber pixy_sub_;
	ros::Subscriber sub_motorDataSet;
	ros::Subscriber sub_rosout;
	ros::Publisher objectCoord_pub;
	//image_transport::Publisher pub_imgStructurePixy;
	//sensor_msgs::ImagePtr imgStructurePixy;
	//std_msgs::Int32MultiArray objCoords;
	//IplImage imgEyeBall;
	//IplImage imgDetectedHead;
	//IplImage structurePixyMerged;

	bool imgEyeBall_received;
	bool imgDetectedHead_received;
	bool structurePixyMerged_received;
	trs_msgs::MotorDataSet motorDataSet;
	bool motorDataSet_arrived;
	rosgraph_msgs::Log rosoutLog;
	bool rosout_arrived;

	trs_msgs::MotorDataSet prev_motorDataSet;
	int encResetCnt[75];

//public:
	//IplImage *imgEyeBall_ptr;
	//IplImage *imgDetectedHead_ptr;
	//IplImage *structurePixyMerged_ptr;

	//sensor_msgs::Image *imgEyeBall_ptr;
	//sensor_msgs::Image *imgDetectedHead_ptr;
	//sensor_msgs::Image *structurePixyMerged_ptr;

};

/*** Main ***/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "trs_cvSurfacePro2GUI_node");
  CvSurfacePro2GUI cvSurfacePro2GUI(ros::this_node::getName());

  //ros::spin();
  while(ros::ok())
  	{
  		ros::spinOnce();
  		//cvReleaseImageHeader(&cvSurfacePro2GUI.imgDetectedHead_ptr);
  		//r.sleep();
  	}


  return 0;
}
