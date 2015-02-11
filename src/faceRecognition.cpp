/*
 * Copyright (c) 2013, The Robot Studio
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
 *  Created on: Feb 5, 2013
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
#include <cv.h> //test
#include <vector>
#include <cvaux.h>
#include <cxcore.hpp>
#include <sys/stat.h>
#include <termios.h>
#include <term.h>
#include <unistd.h>
#include "face_recognition_lib.cpp"
#include <sstream>

/*** Namespaces ***/
using namespace std;

/*** Class ***/
class FaceRecognition
{
public:

  FaceRecognition(std::string name) :
    frl(),
    it_(nh_)
  {
    //cvNamedWindow("Input", CV_WINDOW_AUTOSIZE); 	// output screen
    //cvNamedWindow("Head", CV_WINDOW_NORMAL); 	// CV_WINDOW_NORMAL CV_WINDOW_AUTOSIZE
    cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 1.0, 4.0, 2,2,CV_AA);
    //textColor = CV_RGB(0,255,255);	// light blue text
    show_screen_flag = true;

    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::imageCB, this);
    pub_headCoord = nh_.advertise<std_msgs::Int16MultiArray>("/headCoords", 100);

    pub_imgEyeBall = it_.advertise("/image/imgEyeBall", 1);
    pub_imgDetectedHead = it_.advertise("/image/imgDetectedHead", 1);

    faceMat = cv::imread(ros::package::getPath("trs_vision") + "/resources/search_face.jpg");

    //faceMat = cv::imread("./search_face.jpg");
  }

  ~FaceRecognition(void)
  {
     cvDestroyWindow("Input");
     cvDestroyWindow("Head"); //test
  }

  void imageCB(const sensor_msgs::ImageConstPtr& msg)
  {
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
    ros::Rate r(24);
    IplImage img_input = cv_ptr->image;
    IplImage *img= cvCloneImage(&img_input);
    IplImage *greyImg;
    IplImage *resizedGreyImg;
    IplImage *faceImg;
    IplImage *resizedFaceImg;
    CvRect faceRect;

    // Make sure the image is greyscale, since the Eigenfaces is only done on greyscale image.
    greyImg = frl.convertImageToGreyscale(img);
    //resize down the image :
    resizedGreyImg = frl.resizeImage(greyImg, 320, 240); //HD Ready 720p/4 rez
    // Perform face detection on the input image, using the given Haar cascade classifier.
    faceRect = frl.detectFaceInImage(resizedGreyImg,frl.faceCascade);

    // Make sure a valid face was detected.
    if (faceRect.width >= 1)
    {
    	//multiply by 2, cause rez has been divided once :
    	faceRect.x *= 2;
    	faceRect.y *= 2;
    	faceRect.width *= 2;
    	faceRect.height *= 2;
		//cvRectangle(img, cvPoint(faceRect.x, faceRect.y), cvPoint(faceRect.x + faceRect.width-1, faceRect.y + faceRect.height-1), CV_RGB(0,255,0), 1, 8, 0);

    	//publish head coords :
    	std_msgs::Int16MultiArray head_coords;
    	//Clear array
    	head_coords.data.clear();

    	head_coords.data.push_back(faceRect.x + faceRect.width/2);
    	head_coords.data.push_back(faceRect.y + faceRect.height/2);

    	//test
    	head_coords.data.push_back(faceRect.width);
    	head_coords.data.push_back(faceRect.height);

    	pub_headCoord.publish(head_coords);

		faceImg = frl.cropImage(img, faceRect);	// Get the detected face image.

		//test
		resizedFaceImg = frl.resizeImage(faceImg, 256, 256);
		cv::Mat faceMat = resizedFaceImg;
/*
		ros::Time timeNow = ros::Time::now();
		std::ostringstream oss;
		oss << "/home/dcx/Pictures/Faces/SPS_2014/sps14_" << timeNow.sec << ".jpg";
		
		cv::imwrite(oss.str(), faceMat);*/
		//test

 //HEAD TEST
		cv::Mat scr = faceImg;//resizedFaceImg;//faceImg;
		cv::Mat dst, detected_edges;

		int lowThreshold = 40;
		int ratio = 3;
		int kernel_size = 3;

		/// Reduce noise with a kernel 3x3
		cv::blur( scr, detected_edges, cv::Size(3,3) );
		cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

		for(int row = 0; row < detected_edges.rows; ++row)
		{
			//store gray pointer of edges image
		    unsigned char* inp  = detected_edges.ptr<unsigned char>(row);

		    //store pixels of original image head, but shifted to reach the rectangle.
		    uchar *ptr = (uchar*) (img->imageData + (row+faceRect.y)*img->widthStep);

		    for (int col = 0; col < detected_edges.cols; ++col)
		    {
		        if (*inp++ == 255)
		        {
		        	//Color of edges
		        	ptr[3*(col+faceRect.x)] = 0;//200; //200 //
		        	ptr[3*(col+faceRect.x) + 1] = 255;//100; //120 //green 255
		        	ptr[3*(col+faceRect.x) + 2] = 0;//40; //40 //
		        }
		    }
		}

		imgDetectedHead = cv_bridge::CvImage(std_msgs::Header(), "bgr8", faceImg).toImageMsg();
		pub_imgDetectedHead.publish(imgDetectedHead);

		//cvShowImage("Head", faceImg);
		cvWaitKey(1);
	    cvReleaseImage(&faceImg);
	    //cvReleaseImage(&resizedFaceImg);
    }
    else
    {
    	//publish no head coords but (0;0) instead :
		std_msgs::Int16MultiArray head_coords;
		//Clear array
		head_coords.data.clear();

		head_coords.data.push_back(-1);
		head_coords.data.push_back(-1);
		//test
		head_coords.data.push_back(-1);
		head_coords.data.push_back(-1);

		pub_headCoord.publish(head_coords);

    	//cv::imshow("Head", faceMat);
    	cvWaitKey(1);
    }


    if (show_screen_flag)
    {
    	imgEyeBall = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    	pub_imgEyeBall.publish(imgEyeBall);

		cvWaitKey(1);
    }

    cvReleaseImage(&img);
    cvReleaseImage(&greyImg);
    cvReleaseImage(&resizedGreyImg);
    r.sleep();
    return;
  }


protected:

  FILE *trainFile;
  bool   show_screen_flag;//if output window is shown
  CvFont font;
  //CvScalar textColor;
  ostringstream text_image;
  ros::NodeHandle nh_;
  FaceRecognitionLib frl;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher pub_imgEyeBall;
  image_transport::Publisher pub_imgDetectedHead;
  sensor_msgs::ImagePtr imgEyeBall;
  sensor_msgs::ImagePtr imgDetectedHead;
  ros::Publisher pub_headCoord;
  cv::Mat faceMat;
};

/*** Main ***/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_recognition");
  FaceRecognition face_recognition(ros::this_node::getName());
  ros::spin();
  return 0;
}




