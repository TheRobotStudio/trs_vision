/*
 * Copyright (c) 2012, The Robot Studio
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
 *  Created on: Nov 27, 2012
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

/*** Defines ***/
#define LOOP_RATE				20
#define MAX_NUMBER_PLAYERS		10
#define MAX_WORKING_DISTANCE	2.5 //meters
#define MIN_WORKING_DISTANCE	1

/*** Namespaces ***/
using namespace std;

/*** Variables ***/
string frame_str = "/openni_depth_frame";

bool enableKinectMode = true;

/*** Functions ***/
double calculateAlpha(tf::StampedTransform tf_hand, tf::StampedTransform tf_elbow, tf::StampedTransform tf_shoulder)
{
	double rightHand[3] = {tf_hand.getOrigin().x(), tf_hand.getOrigin().y(), tf_hand.getOrigin().z()};
	double rightElbow[3] = {tf_elbow.getOrigin().x(), tf_elbow.getOrigin().y(), tf_elbow.getOrigin().z()};
	double rightShoulder[3] = {tf_shoulder.getOrigin().x(), tf_shoulder.getOrigin().y(), tf_shoulder.getOrigin().z()};

	//Al-Kashi triangle formula
	double num =  pow(rightHand[0]-rightElbow[0], 2) + pow(rightHand[1]-rightElbow[1], 2) + pow(rightHand[2]-rightElbow[2], 2)
				+ pow(rightShoulder[0]-rightElbow[0], 2) + pow(rightShoulder[1]-rightElbow[1], 2) + pow(rightShoulder[2]-rightElbow[2], 2)
				- pow(rightShoulder[0]-rightHand[0], 2) - pow(rightShoulder[1]-rightHand[1], 2) - pow(rightShoulder[2]-rightHand[2], 2);

	double den = 2 * sqrt(pow(rightHand[0]-rightElbow[0], 2) + pow(rightHand[1]-rightElbow[1], 2) + pow(rightHand[2]-rightElbow[2], 2)) * sqrt(pow(rightShoulder[0]-rightElbow[0], 2) + pow(rightShoulder[1]-rightElbow[1], 2) + pow(rightShoulder[2]-rightElbow[2], 2));

	return acos(num/den);
}

double calculateBeta(tf::StampedTransform tf_elbow, tf::StampedTransform tf_neck, tf::StampedTransform tf_shoulder)
{
	double rightElbow[3] = {tf_elbow.getOrigin().x(), tf_elbow.getOrigin().y(), tf_elbow.getOrigin().z()};
	double neck[3] = {tf_neck.getOrigin().x(), tf_neck.getOrigin().y(), tf_neck.getOrigin().z()};
	double rightShoulder[3] = {tf_shoulder.getOrigin().x(), tf_shoulder.getOrigin().y(), tf_shoulder.getOrigin().z()};

	//Al-Kashi triangle formula
	double num =  pow(rightElbow[0]-rightShoulder[0], 2) + pow(rightElbow[1]-rightShoulder[1], 2) + pow(rightElbow[2]-rightShoulder[2], 2)
						+ pow(rightShoulder[0]-neck[0], 2) + pow(rightShoulder[1]-neck[1], 2) + pow(rightShoulder[2]-neck[2], 2)
						- pow(neck[0]-rightElbow[0], 2) - pow(neck[1]-rightElbow[1], 2) - pow(neck[2]-rightElbow[2], 2);

	double den = 2 * sqrt(pow(rightElbow[0]-rightShoulder[0], 2) + pow(rightElbow[1]-rightShoulder[1], 2) + pow(rightElbow[2]-rightShoulder[2], 2)) * sqrt(pow(rightShoulder[0]-neck[0], 2) + pow(rightShoulder[1]-neck[1], 2) + pow(rightShoulder[2]-neck[2], 2));

	return acos(num/den);
}

double calculateTheta(tf::StampedTransform tf_elbow, tf::StampedTransform tf_neck, tf::StampedTransform tf_shoulder,  tf::StampedTransform tf_torso)
{
	double rightElbow[3] = {tf_elbow.getOrigin().x(), tf_elbow.getOrigin().y(), tf_elbow.getOrigin().z()};
	double neck[3] = {tf_neck.getOrigin().x(), tf_neck.getOrigin().y(), tf_neck.getOrigin().z()};
	double rightShoulder[3] = {tf_shoulder.getOrigin().x(), tf_shoulder.getOrigin().y(), tf_shoulder.getOrigin().z()};
	double torso[3] = {tf_torso.getOrigin().x(), tf_torso.getOrigin().y(), tf_torso.getOrigin().z()};

	////Theta is the angle between two planes : TNE and SNE
	double a1 = (rightShoulder[1]-torso[1])*(neck[2]-torso[2]) - (rightShoulder[2]-torso[2])*(neck[1]-torso[1]);
	double b1 = (rightShoulder[2]-torso[2])*(neck[0]-torso[0]) - (rightShoulder[0]-torso[0])*(neck[2]-torso[2]);
	double c1 = (rightShoulder[0]-torso[0])*(neck[1]-torso[1]) - (rightShoulder[1]-torso[1])*(neck[0]-torso[0]);

	double a2 = (rightShoulder[1]-rightElbow[1])*(neck[2]-rightElbow[2]) - (rightShoulder[2]-rightElbow[2])*(neck[1]-rightElbow[1]);
	double b2 = (rightShoulder[2]-rightElbow[2])*(neck[0]-rightElbow[0]) - (rightShoulder[0]-rightElbow[0])*(neck[2]-rightElbow[2]);
	double c2 = (rightShoulder[0]-rightElbow[0])*(neck[1]-rightElbow[1]) - (rightShoulder[1]-rightElbow[1])*(neck[0]-rightElbow[0]);

	double num = a1*a2 + b1*b2 + c1*c2;

	double den = sqrt((a1*a1+b1*b1+c1*c1)*(a2*a2+b2*b2+c2*c2));

	return acos(num/den);
}

double calculatePhi(tf::StampedTransform tf_hand, tf::StampedTransform tf_neck, tf::StampedTransform tf_shoulder,  tf::StampedTransform tf_elbow)
{
	double H[3] = {tf_hand.getOrigin().x(), tf_hand.getOrigin().y(), tf_hand.getOrigin().z()};
	double N[3] = {tf_neck.getOrigin().x(), tf_neck.getOrigin().y(), tf_neck.getOrigin().z()};
	double S[3] = {tf_shoulder.getOrigin().x(), tf_shoulder.getOrigin().y(), tf_shoulder.getOrigin().z()};
	double E[3] = {tf_elbow.getOrigin().x(), tf_elbow.getOrigin().y(), tf_elbow.getOrigin().z()};

	//Phi is the angle between two planes : SNE and SHE
	float n1[3] = {(N[1]-S[1])*(E[2]-S[2])-(N[2]-S[2])*(E[1]-S[1]),
					(N[2]-S[2])*(E[0]-S[0])-(N[0]-S[0])*(E[2]-S[2]),
					(N[0]-S[0])*(E[1]-S[1])-(N[1]-S[1])*(E[0]-S[0])}; //normal plane vector : SNE

	float n2[3] = {(H[1]-S[1])*(E[2]-S[2])-(H[2]-S[2])*(E[1]-S[1]),
						(H[2]-S[2])*(E[0]-S[0])-(H[0]-S[0])*(E[2]-S[2]),
						(H[0]-S[0])*(E[1]-S[1])-(H[1]-S[1])*(E[0]-S[0])}; //normal plane vector : SHE

	float a1,b1,c1;
	a1 = n1[0];
	b1 = n1[1];
	c1 = n1[2];

	float a2,b2,c2;
	a2 = n2[0];
	b2 = n2[1];
	c2 = n2[2];

	double angle_phi = acos(fabs((a1*a2 + b1*b2 + c1*c2)/(sqrt((a1*a1+b1*b1+c1*c1)*(a2*a2+b2*b2+c2*c2)))));

	//std::cerr << "angle_phi" << angle_phi << std::endl;

	return angle_phi;
}

/*** Callback functions ***/
void tabletCmd_cb(const std_msgs::BoolConstPtr& tablet_cmd)
{
	enableKinectMode = tablet_cmd->data;
}

/*** Main ***/
int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "trs_kinectToAngles_node");
	ros::NodeHandle nh;

	//Publishers
	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/anglesArmsDescription", 100);

	//Subscribers
	//subscribe to enable or disable kinect mode
	ros::Subscriber tabletCmd_kinect_sub = nh.subscribe("/tabletCmd_kinect", 1, tabletCmd_cb);

	ros::Rate loop_rate(LOOP_RATE);

	tf::TransformListener lst_neck, lst_torso, lst_R_shoulder, lst_R_elbow, lst_R_hand, lst_L_shoulder, lst_L_elbow, lst_L_hand;
	tf::StampedTransform tf_neck, tf_torso, tf_R_shoulder, tf_R_elbow, tf_R_hand, tf_L_shoulder, tf_L_elbow, tf_L_hand;

	// message declarations
	sensor_msgs::JointState joint_state;

	float prev_torso_depth[MAX_NUMBER_PLAYERS] = {0}; //in meters
	bool torsoFound = false;
	bool playerValid[MAX_NUMBER_PLAYERS] = {false};
	float player_distance = 10;

	while (ros::ok())
	{
		if(enableKinectMode)
		{
			std::ostringstream playerNb_str;
			int playerNb_int = 0;

			torsoFound = false;
			playerValid[MAX_NUMBER_PLAYERS] = {false};
			player_distance = 10;

			//first check how many players are detected
			for(int i=1; i<=MAX_NUMBER_PLAYERS; i++)
			{
				try
				{
					std::ostringstream nb_oss;
					nb_oss << i;
					//ROS_INFO("playerNb_str %s ", playerNb_str);
					//playerNb_str
					lst_torso.lookupTransform(frame_str, "/torso_" + nb_oss.str(), ros::Time(0), tf_torso);
					torsoFound = true;
				}
				catch (tf::TransformException ex)
				{
					//ROS_ERROR("%s",ex.what());
					torsoFound = false;
				}

				if(torsoFound)
				{
					float torso_depth = tf_torso.getOrigin().x();

					//update if player is valid for this loop
					if((prev_torso_depth[i] != torso_depth) && (torso_depth <= MAX_WORKING_DISTANCE) && (torso_depth >= MIN_WORKING_DISTANCE))
					{
						playerValid[i] = true;

						//update to find the nearest player to the kinect
						if(torso_depth < player_distance)
						{
							player_distance = torso_depth;
							//erase the playerNb_str
							//playerNb_str.clear();//flush();
							//playerNb_str.str("");

							//update
							playerNb_int = i;
						}

						//ROS_INFO("Player %d valid - distance = %f", i, torso_depth);
					}
				/*	else
					{
						ROS_INFO("Player %d not valid - distance = %f", i, torso_depth);
					}
	*/
					//update with the torso_depth
					prev_torso_depth[i] = torso_depth;
				}
				//else ROS_INFO("Player %d doesn't exist !", i);
			}//for

			playerNb_str << playerNb_int;
			//cout << "playerNb_str = " << playerNb_str.str() << endl;

			if(playerNb_int != 0)
			{
				//ROS_INFO("Player %d has been selected", playerNb_int);

				//grab neck and torso
				try
				{
					lst_neck.lookupTransform(frame_str, "/neck_" + playerNb_str.str(), ros::Time(0), tf_neck);
				}
				catch (tf::TransformException ex)
				{
					//ROS_ERROR("%s",ex.what());
				}

				try
				{
					lst_torso.lookupTransform(frame_str, "/torso_" + playerNb_str.str(), ros::Time(0), tf_torso);
				}
				catch (tf::TransformException ex)
				{
					//ROS_ERROR("%s",ex.what());
				}

				//grab right tf skeleton joints (using left tf in fact)
				//chose left for RIGHT ARM and right for LEFT ARM

				try
				{
					lst_R_shoulder.lookupTransform(frame_str, "/left_shoulder_" + playerNb_str.str(), ros::Time(0), tf_R_shoulder);
				}
				catch (tf::TransformException ex)
				{
					//ROS_ERROR("%s",ex.what());
				}

				try
				{
					lst_R_elbow.lookupTransform(frame_str, "/left_elbow_" + playerNb_str.str(), ros::Time(0), tf_R_elbow);
				}
				catch (tf::TransformException ex)
				{
					//ROS_ERROR("%s",ex.what());
				}

				try
				{
					lst_R_hand.lookupTransform(frame_str, "/left_hand_" + playerNb_str.str(), ros::Time(0), tf_R_hand);
				}
				catch (tf::TransformException ex)
				{
					//ROS_ERROR("%s",ex.what());
				}

				//grab left tf skeleton joints (using right tf in fact)
				//(chose left for RIGHT ARM and right for LEFT ARM)

				try
				{
					lst_L_shoulder.lookupTransform(frame_str, "/right_shoulder_" + playerNb_str.str(), ros::Time(0), tf_L_shoulder);
				}
				catch (tf::TransformException ex)
				{
					//ROS_ERROR("%s",ex.what());
				}

				try
				{
					lst_L_elbow.lookupTransform(frame_str, "/right_elbow_" + playerNb_str.str(), ros::Time(0), tf_L_elbow);
				}
				catch (tf::TransformException ex)
				{
					//ROS_ERROR("%s",ex.what());
				}

				try
				{
					lst_L_hand.lookupTransform(frame_str, "/right_hand_" + playerNb_str.str(), ros::Time(0), tf_L_hand);
				}
				catch (tf::TransformException ex)
				{
					//ROS_ERROR("%s",ex.what());
				}

				//update joint_state
				//if using the other definition of the angles : beta and theta defined with the hand instead of the elbow
				joint_state.header.stamp = ros::Time::now();
				joint_state.name.resize(8);
				joint_state.position.resize(8);
/*
				//left arm
				joint_state.name[0] ="left_elbow_alpha_joint";
				joint_state.position[0] = M_PI - calculateAlpha(tf_L_hand, tf_L_elbow, tf_L_shoulder);
				joint_state.name[1] ="left_shoulder_beta_joint";
				joint_state.position[1] = M_PI - calculateBeta(tf_L_hand, tf_neck, tf_L_shoulder);
				joint_state.name[2] ="left_shoulder_theta_joint";
				joint_state.position[2] = calculateTheta(tf_L_hand, tf_neck, tf_L_shoulder, tf_torso) - M_PI/2;
				joint_state.name[3] ="left_shoulder_phi_joint";
				joint_state.position[3] = calculatePhi(tf_L_hand, tf_neck, tf_L_shoulder, tf_L_elbow);

				//right arm
				joint_state.name[4] ="right_elbow_alpha_joint";
				joint_state.position[4] = M_PI - calculateAlpha(tf_R_hand, tf_R_elbow, tf_R_shoulder);
				joint_state.name[5] ="right_shoulder_beta_joint";
				joint_state.position[5] = M_PI - calculateBeta(tf_R_hand, tf_neck, tf_R_shoulder);
				joint_state.name[6] ="right_shoulder_theta_joint";
				joint_state.position[6] = calculateTheta(tf_R_hand, tf_neck, tf_R_shoulder, tf_torso) - M_PI/2;
				joint_state.name[7] ="right_shoulder_phi_joint";
				joint_state.position[7] = calculatePhi(tf_R_hand, tf_neck, tf_R_shoulder, tf_R_elbow);
*/
				//test to invert arms
				//left arm
				joint_state.name[4] ="left_elbow_alpha_joint";
				joint_state.position[4] = M_PI - calculateAlpha(tf_L_hand, tf_L_elbow, tf_L_shoulder);
				joint_state.name[5] ="left_shoulder_beta_joint";
				joint_state.position[5] = M_PI - calculateBeta(tf_L_hand, tf_neck, tf_L_shoulder);
				joint_state.name[6] ="left_shoulder_theta_joint";
				joint_state.position[6] = calculateTheta(tf_L_hand, tf_neck, tf_L_shoulder, tf_torso) - M_PI/2;
				joint_state.name[7] ="left_shoulder_phi_joint";
				joint_state.position[7] = calculatePhi(tf_L_hand, tf_neck, tf_L_shoulder, tf_L_elbow);

				//right arm
				joint_state.name[0] ="right_elbow_alpha_joint";
				joint_state.position[0] = M_PI - calculateAlpha(tf_R_hand, tf_R_elbow, tf_R_shoulder);
				joint_state.name[1] ="right_shoulder_beta_joint";
				joint_state.position[1] = M_PI - calculateBeta(tf_R_hand, tf_neck, tf_R_shoulder);
				joint_state.name[2] ="right_shoulder_theta_joint";
				joint_state.position[2] = calculateTheta(tf_R_hand, tf_neck, tf_R_shoulder, tf_torso) - M_PI/2;
				joint_state.name[3] ="right_shoulder_phi_joint";
				joint_state.position[3] = calculatePhi(tf_R_hand, tf_neck, tf_R_shoulder, tf_R_elbow);
			}
			else //if no player found
			{
				//ROS_INFO("No player found");

				//resize 0 with no data
				joint_state.header.stamp = ros::Time::now();
				joint_state.name.resize(0);
				joint_state.position.resize(0);
			}

			//send the joint state and transform
			joint_pub.publish(joint_state);
		}//if enable

		ros::spinOnce();	
		loop_rate.sleep();
	}
}
