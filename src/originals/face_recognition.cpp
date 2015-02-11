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
    cvNamedWindow("Input", CV_WINDOW_AUTOSIZE); 	// output screen
    cvNamedWindow("Head", CV_WINDOW_NORMAL); 	// CV_WINDOW_NORMAL CV_WINDOW_AUTOSIZE
    cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 1.0, 4.0, 2,2,CV_AA);
    textColor = CV_RGB(0,255,255);	// light blue text
    show_screen_flag = true;

    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::imageCB, this);
    pub_headCoord = nh_.advertise<std_msgs::Int16MultiArray>("/headCoords", 100);

    faceMat = cv::imread("/home/dcx/Pictures/resources/search_face.jpg");
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

    	pub_headCoord.publish(head_coords);

		faceImg = frl.cropImage(img, faceRect);	// Get the detected face image.

		//test
		resizedFaceImg = frl.resizeImage(faceImg, 256, 256);
		cv::Mat faceMat = resizedFaceImg;
		ros::Time timeNow = ros::Time::now();
		std::ostringstream oss;
		oss << "/home/dcx/Pictures/Medtech_2013_Luzern/medtech13_" << timeNow.sec << ".jpg";
		//string fileName = ; //+
		cv::imwrite(oss.str(), faceMat);
		//test

 //HEAD TEST
		cv::Mat scr = faceImg;//resizedFaceImg;//faceImg;
		cv::Mat dst, detected_edges;

		//int edgeThresh = 1;
		int lowThreshold = 40;
		//int const max_lowThreshold = 100;
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
		        	ptr[3*(col+faceRect.x)] = 0; //200
		        	ptr[3*(col+faceRect.x) + 1] = 255; //120
		        	ptr[3*(col+faceRect.x) + 2] = 0; //40
		        }
		    }
		}
	/*
		for(int row = 0; row < detected_edges.rows; ++row)
		{
			//store gray pointer of edges image
			unsigned char* inp  = detected_edges.ptr<unsigned char>(row);

			//store pixels of colored head
			uchar *ptr = (uchar*) (resizedFaceImg->imageData + row*resizedFaceImg->widthStep);

			for (int col = 0; col < detected_edges.cols; ++col)
			{
				if (*inp++ == 255)
				{
					ptr[3*col] = 0;
					ptr[3*col + 1] = 255;
					ptr[3*col + 2] = 0;
				}
			}
		}
*/
		//cv::cvtColor(detected_edges, dst, CV_GRAY2RGB);
		//cv::add(detected_edges, dst, dst);
/*
		//add two images:
		double alpha = 0.8; double beta;
		beta = ( 1.0 - alpha );
		cv::addWeighted(scr, alpha, dst, beta, 0.0, dst);
*/
		//cv::imshow("Head", dst);
		//fin test

/*
		//test
		//cvSetImageROI(img, faceRect);
		cv::Mat scr = img;
		cv::Mat dst, detected_edges;

		//int edgeThresh = 1;
		int lowThreshold = 50;
		//int const max_lowThreshold = 100;
		int ratio = 3;
		int kernel_size = 3;

		/// Reduce noise with a kernel 3x3
		cv::blur( scr, detected_edges, cv::Size(3,3) );
		cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

		cv::imshow("Head", detected_edges);
		//fin test
*/

		cvShowImage("Head", faceImg);
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

		pub_headCoord.publish(head_coords);

    	cv::imshow("Head", faceMat);
    	cvWaitKey(1);
    }


    if (show_screen_flag)
    {
		//test add blend image
		/// Read image ( same size, same type )
    	/*
		cv::Mat src1, dst;
		cv::Mat scr2 = img;
		*/
		//cv::Mat cropROI = scr2(faceRect);
		//cv::Canny(cropROI, cropROI, 80, 240, 3);
		//cv::Canny(scr2, scr2, 16, 64, 3);
/*
		src1 = cv::imread("/home/dcx/Pictures/screen-pattern.jpg");

		double alpha = 0.1; double beta;
		beta = ( 1.0 - alpha );
		cv::addWeighted(src1, alpha, scr2, beta, 0.0, dst);
*/
    	//cv::Mat faceMat = faceImg;
    	//if(faceImg)

		//cv::imshow("Input", dst);
		cvShowImage("Input", img);
		cvWaitKey(1);
    }

    cvReleaseImage(&img);
    cvReleaseImage(&greyImg);
    cvReleaseImage(&resizedGreyImg);
    //cvReleaseImage(src1); cvReleaseImage(&src2);
    r.sleep();
    return;
  }


protected:

  FILE *trainFile;
  bool   show_screen_flag;//if output window is shown
  CvFont font;
  CvScalar textColor;
  ostringstream text_image;
  ros::NodeHandle nh_;
  FaceRecognitionLib frl;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher pub_headCoord;
  cv::Mat faceMat;// = cv::imread("/home/dcx/Pictures/search_face.jpg");
};

/*** Main ***/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_recognition");
  FaceRecognition face_recognition(ros::this_node::getName());
  ros::spin();
  return 0;
}




