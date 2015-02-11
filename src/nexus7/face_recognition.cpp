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
#include "opencv2/objdetect/objdetect.hpp"
#include <cv.h> //test
#include <vector>
#include <cvaux.h>
#include <cxcore.hpp>
#include <sys/stat.h>
#include <termios.h>
#include <term.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <stdio.h>

/*** Namespaces ***/
using namespace std;
using namespace cv;

/*** Function Headers ***/
CvRect detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade );

/*** Global variables ***/
static const char * face_cascade_name = "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
CvHaarClassifierCascade* faceCascade;

ros::Publisher pub_headCoord;

/*** @function main ***/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "trs_vision_node");
	ros::NodeHandle n;
	CvRect faceRect;
	Point center;
	std_msgs::Int16MultiArray head_coords;

	double t; //time measurement

	head_coords.data.clear();
	head_coords.data.push_back(-1);
	head_coords.data.push_back(-1);

	//namedWindow("image1", CV_WINDOW_AUTOSIZE);
	//ros::Rate r(15);
	pub_headCoord = n.advertise<std_msgs::Int16MultiArray>("/headCoords", 10);

	// Load the HaarCascade classifier for face detection.
	faceCascade = (CvHaarClassifierCascade*)cvLoad(face_cascade_name, 0, 0, 0 );
	if(!faceCascade)
	{
		ROS_INFO("Could not load Haar cascade Face detection classifier in '%s'.", face_cascade_name);
		exit(1);
	}

	VideoCapture cap = VideoCapture(1); //-1

	while( ros::ok() )
	{
		t = (double)cvGetTickCount();

		if(waitKey(10) >= 0 ) break; //50

		Mat frameMat;

		if(!cap.grab()){
			printf("failed to grab from camera\n");
			break;
		}

		cap >> frameMat;

		if(frameMat.empty()){
			printf("failed to grab from camera\n");
			break;
		}



		resize((InputArray)frameMat, (OutputArray)frameMat, Size(), 0.5, 0.5, CV_INTER_AREA);

		cvtColor(frameMat, frameMat, CV_BGR2GRAY); //convert to greyscale
		IplImage image2 = frameMat;



		faceRect = detectFaceInImage(&image2, faceCascade);

		t = (double)cvGetTickCount() - t;
		ROS_INFO("[Time measure is %d ms]\n", cvRound( t/((double)cvGetTickFrequency()*1000.0) ));


		// Make sure a valid face was detected.
		if (faceRect.width >= 1)
		{
			//multiply by 2, cause rez has been divided once :
			faceRect.x *= 2;
			faceRect.y *= 2;
			faceRect.width *= 2;
			faceRect.height *= 2;

			//overwrite head coords :
			head_coords.data[0] = faceRect.x + faceRect.width/2;
			head_coords.data[1] = faceRect.y + faceRect.height/2;
		}
		else
		{
			//overwrite head coords with (-1;-1):
			head_coords.data[0] = -1;
			head_coords.data[1] = -1;
		}

		//publish head coords
		pub_headCoord.publish(head_coords);

		//Point center( (faceRect.x + faceRect.width*0.5)/2, (faceRect.y + faceRect.height*0.5)/2 );
		center.x = (faceRect.x + faceRect.width*0.5)/2;
		center.y = (faceRect.y + faceRect.height*0.5)/2;
		ellipse(frameMat, center, Size( faceRect.width*0.25, faceRect.height*0.25), 0, 0, 360, Scalar(0, 255, 0), 4, 8, 0);

		imshow("FaceTracking", frameMat);

		frameMat.release();
		//r.sleep();
	}

	//waitKey(0);
	//cvDestroyWindow("Face Tracking");
	return 0;
}

/*** Functions ***/
CvRect detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade )
{
 	const CvSize minFeatureSize = cvSize(10, 10);
 	const int flags = CV_HAAR_FIND_BIGGEST_OBJECT; //CV_HAAR_DO_CANNY_PRUNING CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_DO_ROUGH_SEARCH;	// Only search for 1 face.
 	const float search_scale_factor = 1.2f; //1.1f;
 	CvMemStorage* storage;
 	CvRect rc;
 	//double t;
 	CvSeq* rects;

 	storage = cvCreateMemStorage(0);
 	cvClearMemStorage( storage );

 	// Detect all the faces.
 	//t = (double)cvGetTickCount();  //below change to detectimg
 	rects = cvHaarDetectObjects( inputImg, (CvHaarClassifierCascade*)cascade, storage,
 				search_scale_factor, 2, flags, minFeatureSize ); //3
 	//t = (double)cvGetTickCount() - t;
 	//ROS_INFO("[Face Detection took %d ms and found %d objects]\n", cvRound( t/((double)cvGetTickFrequency()*1000.0) ), rects->total );

 	// Get the first detected face (the biggest).
 	if (rects->total > 0)
         {
           rc = *(CvRect*)cvGetSeqElem( rects, 0 );
         }
 	else
 		rc = cvRect(-1,-1,-1,-1);	// Couldn't find the face.

 	return rc;	// Return the biggest face found, or (-1,-1,-1,-1).
}
