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
#include <std_msgs/Int16MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int32MultiArray.h>
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

/*** Defines ***/
#define LOOP_RATE 	50

/*** Namespaces ***/
using namespace std;
using namespace cv;

/*** Class ***/
class StructureSensor
{
public:

	StructureSensor(std::string name) :
    it_(nh_)
	{
		//cvNamedWindow("Pixy and Structure Fusion", CV_WINDOW_AUTOSIZE); 	// output screen
		cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 1.0, 4.0, 2,2,CV_AA);
		textColor = CV_RGB(0,255,255);

		//Image transport Subscribers and Publishers
		image_sub_ = it_.subscribe("/depth/image", 1, &StructureSensor::structureSensor_cb, this);
		pub_imgStructurePixy = it_.advertise("/image/structurePixyMerged", 1);

		//Nodehandle Subscribers and Publishers
		pixy_sub_ = nh_.subscribe("/pixy", 1, &StructureSensor::pixy_cb, this);
		objectCoord_pub = nh_.advertise<std_msgs::Int32MultiArray>("/objectCoord", 1);

		objCoords.data.clear();
		//fill with data, 2 objects
		objCoords.data.push_back(0);
		objCoords.data.push_back(0);
		objCoords.data.push_back(0);
		objCoords.data.push_back(0);
		objCoords.data.push_back(0);
		objCoords.data.push_back(0);

		for(int i=0; i<7; i++)
		objectDetected[i] = false;;
	}

	~StructureSensor(void)
	{
		cvDestroyWindow("Pixy and Structure Fusion");
	}

	void structureSensor_cb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		//convert from ros image format to opencv image format
		try
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, "32FC1"); //"bgr8"
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}
		ros::Rate r(LOOP_RATE);
		IplImage img_input = cv_ptr->image;
		IplImage *img = cvCloneImage(&img_input);

		for(int i=0; i<2; i++)
		{
			if(objectDetected[i] && (objectRect[i].height >15) && (objectRect[i].width >15) && (objectRect[i].x >10) && (objectRect[i].x <630) && (objectRect[i].y >10) && (objectRect[i].y <470))
			{
				CvRect objectROIRect = objectRect[i];
				objectROIRect.width = 10;
				objectROIRect.height = 10;

				//ROI
				IplImage *imgROI = cropImage(img, objectROIRect);
				Mat src = img; //imgROI;

				IplImage *imgROI_8u = cvCreateImage(cvSize(imgROI->width, imgROI->height),IPL_DEPTH_8U,1)  ;
				cvConvertScale(imgROI, imgROI_8u, 255.);
				cv::Mat imgROIMat = imgROI_8u;
				objectDepth[i] = getMaxHist(imgROIMat);
				//showHistogram(imgROIMat);
	/*
				float depth_cm_f = (float)objectDepth;
				depth_cm_f *= 0.394;
	*/
				if(objectDepth[i] != 0) //black means no data
				{
					char depth_c[32];
					sprintf(depth_c, "%d", objectDepth[i]);
					//sprintf(depth_c, "%f", depth_cm_f);

					//modify slightly the rectangle with the depth
					objectRect[i].x -= (150 - objectDepth[i])*0.35;

					//draw
					cvRectangle(img, cvPoint(objectRect[i].x - objectRect[i].width/2, objectRect[i].y - objectRect[i].height/2), cvPoint(objectRect[i].x + objectRect[i].width/2, objectRect[i].y + objectRect[i].height/2), CV_RGB(0,255,0), 2+i, 8, 0);
					cvPutText(img, depth_c, cvPoint(objectRect[i].x + objectRect[i].width/2, objectRect[i].y + objectRect[i].height/2), &font, CV_RGB(0,0,0));

					//fill with data
					objCoords.data[i*3+0] = objectRect[i].x;
					objCoords.data[i*3+1] = objectRect[i].y;
					objCoords.data[i*3+2] = objectDepth[i];
				}
				else
				{
					//fill with data
					objCoords.data[i*3+0] = 0;
					objCoords.data[i*3+1] = 0;
					objCoords.data[i*3+2] = 0;
				}
			}
			else
			{
				//fill with data
				objCoords.data[i*3+0] = 0;
				objCoords.data[i*3+1] = 0;
				objCoords.data[i*3+2] = 0;
			}
		}

		objectCoord_pub. publish(objCoords);

		imgStructurePixy = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		pub_imgStructurePixy.publish(imgStructurePixy);

		//cvShowImage("Pixy and Structure Fusion", img);
		//cvWaitKey(1);

		cvReleaseImage(&img);
		r.sleep();
		return;
	}

	void pixy_cb(const std_msgs::Int32MultiArrayConstPtr& msg)
	{
		for(int i=0; i<7; i++)
		objectDetected[i] = false;

		if(msg->data[0] == 1) //color code 10 = 12 in octal, for combination of color code 1 and 2, always given with smaller code first
		{
			objectDetected[0] = true;

			objectRect[0].height = msg->data[4]*2;
			objectRect[0].width = msg->data[3]*2;
			objectRect[0].y = msg->data[2]*2 + 45; //+30 for old setup, pixy on top of structure
			objectRect[0].x = msg->data[1]*2 - 45; //-5 for old setup, pixy on top of structure

			//linear scaling for the Pixy lens to match the structure sensor
			objectRect[0].x = objectRect[0].x - (320-objectRect[0].x)*0.25;
			objectRect[0].y = objectRect[0].y - (240-objectRect[0].y)*0.25;
		}

		if(msg->data[5] == 1) //signature object == 2
		{
			objectDetected[1] = true;

			objectRect[1].height = msg->data[4+5]*2;
			objectRect[1].width = msg->data[3+5]*2;
			objectRect[1].y = msg->data[2+5]*2 + 45;
			objectRect[1].x = msg->data[1+5]*2 - 45; //35

			//linear scaling for the Pixy lens to match the structure sensor
			objectRect[1].x = objectRect[1].x - (320-objectRect[1].x)*0.25;
			objectRect[1].y = objectRect[1].y - (240-objectRect[1].y)*0.25;
		}
	}

	IplImage* cropImage(const IplImage *img, const CvRect region)
	{
		IplImage *imageTmp;
		IplImage *imageCrop;
		CvSize size;
		size.height = img->height;
		size.width = img->width;

		// First create a new greyscale IPL Image and copy contents of img into it.
		imageTmp = cvCreateImage(size, IPL_DEPTH_32F, img->nChannels);
		cvCopy(img, imageTmp, NULL);

		// Create a new image of the detected region
		// Set region of interest to that surrounding the face
		cvSetImageROI(imageTmp, region);
		// Copy region of interest (i.e. face) into a new iplImage (imageRGB) and return it
		size.width = region.width;
		size.height = region.height;
		imageCrop = cvCreateImage(size, IPL_DEPTH_32F, img->nChannels);
		cvCopy(imageTmp, imageCrop, NULL);	// Copy just the region.

	    cvReleaseImage( &imageTmp );
		return imageCrop;
	}
/*
	void showHistogram(Mat& img)
	{
	    long bins = 256;            // number of bins
	    int nc = img.channels();    // number of channels
	    vector<Mat> hist(nc);       // array for storing the histograms
	    vector<Mat> canvas(nc);     // images for displaying the histogram
	    int hmax[3] = {0,0,0};      // peak value for each histogram

	    // The rest of the code will be placed here
	    for (int i = 0; i < hist.size(); i++)
	        hist[i] = Mat::zeros(1, bins, CV_32FC1);

	    for (int i = 0; i < img.rows; i++)
	    {
	        for (int j = 0; j < img.cols; j++)
	        {
	            for (int k = 0; k < nc; k++)
	            {
	                uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<Vec3b>(i,j)[k];
	                hist[k].at<int>(val) += 1;
	            }
	        }
	    }

	    for (int i = 0; i < nc; i++)
	    {
	        for (int j = 0; j < bins-1; j++)
	        {
	            //hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
	        	if(hist[i].at<int>(j) > hmax[i])
				{
	        		hmax[i] = hist[i].at<int>(j);
	        		objectDepth = j;
				}
	        }
	    }

	    //ROS_INFO("objectDepth = %d", objectDepth);
	    //ROS_INFO("hmax = %d, j = %d", hmax[i], j);

	    const char* wname[3] = { "blue", "green", "red" };
	    Scalar colors[3] = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };

	    for (int i = 0; i < nc; i++)
	    {
	        canvas[i] = Mat::ones(125, bins, CV_8UC3);

	        for (int j = 0, rows = canvas[i].rows; j < bins-1; j++)
	        {
	        	if(hmax[i] != 0)
	            {
	        		line(
						canvas[i],
						Point(j, rows),
						Point(j, rows - (hist[i].at<int>(j) * rows/hmax[i])),
						nc == 1 ? Scalar(200,200,200) : colors[i],
						1, 8, 0
	        		);
	            }
	        }

	        imshow(nc == 1 ? "Center Area Histogram" : wname[i], canvas[i]);
	    }

	}
*/
	int getMaxHist(Mat& img)
	{
		long bins = 256;            // number of bins
		int nc = img.channels();    // number of channels
		vector<Mat> hist(nc);       // array for storing the histograms
		vector<Mat> canvas(nc);     // images for displaying the histogram
		int hmax[3] = {0,0,0};      // peak value for each histogram
		int objDepth = 0;

		// The rest of the code will be placed here
		for (int i = 0; i < hist.size(); i++)
			hist[i] = Mat::zeros(1, bins, CV_32FC1);

		for (int i = 0; i < img.rows; i++)
		{
			for (int j = 0; j < img.cols; j++)
			{
				for (int k = 0; k < nc; k++)
				{
					uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<Vec3b>(i,j)[k];
					hist[k].at<int>(val) += 1;
				}
			}
		}

		for (int i = 0; i < nc; i++)
		{
			for (int j = 0; j < bins-1; j++)
			{
				//hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
				if(hist[i].at<int>(j) > hmax[i])
				{
					hmax[i] = hist[i].at<int>(j);
					objDepth = j;
				}
			}
		}

		return objDepth;
	}

protected:

	ros::NodeHandle nh_;
	CvFont font;
    CvScalar textColor;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber pixy_sub_;
	ros::Publisher objectCoord_pub;
	image_transport::Publisher pub_imgStructurePixy;
	sensor_msgs::ImagePtr imgStructurePixy;
	std_msgs::Int32MultiArray objCoords;
	CvRect objectRect[7];
	int objectDepth[7];
	bool objectDetected[7];
};

/*** Main ***/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "trs_stuctureSensor_node");
  StructureSensor structure_sensor(ros::this_node::getName());
  ros::spin();
  return 0;
}
