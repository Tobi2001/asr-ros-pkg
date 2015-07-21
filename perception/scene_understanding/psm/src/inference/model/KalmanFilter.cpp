/*
 * KalmanFilter.cpp
 *
 *  Created on: 14.05.2014
 *      Author: schleich
 */

#include <iostream>

#include "inference/model/KalmanFilter.h"

namespace ProbabilisticSceneRecognition {
	KalmanFilter::KalmanFilter(pbd_msgs::PbdObject pObject) :
			mReset(true)
	{
	  	mF = Eigen::MatrixXd::Identity(7, 7);
		mH = Eigen::MatrixXd::Identity(7, 7);

		mQ = 0.1 * Eigen::MatrixXd::Identity(7, 7);
		mR = 1 * Eigen::MatrixXd::Identity(7, 7);
		
		mP = Eigen::MatrixXd::Identity(mF.rows(), mF.cols());
		
		update(pObject);
	}

	// dtor
	KalmanFilter::~KalmanFilter() {
	}

	void KalmanFilter::reset() {
		mReset = true;
	}

	void KalmanFilter::update(pbd_msgs::PbdObject pObject) {
	  
		// Bring last update time uptodate.
		lastUpdate = std::chrono::high_resolution_clock::now();
	  
		// Rename the mesaurement to x.
		Eigen::VectorXd x = Eigen::VectorXd::Zero(7);
		x(0) = pObject.poseEstimation.pose.position.x;
		x(1) = pObject.poseEstimation.pose.position.y;
		x(2) = pObject.poseEstimation.pose.position.z;
		x(3) = pObject.poseEstimation.pose.orientation.w;
		x(4) = pObject.poseEstimation.pose.orientation.x;
		x(5) = pObject.poseEstimation.pose.orientation.y;
		x(6) = pObject.poseEstimation.pose.orientation.z;
		
		// If reset flag is true, reset the system to the current measurement.
		if (mReset) {
			mReset = false;
			mX = x;
			mZ = mH * x;
		}

		// Prediction step.
		Eigen::VectorXd xPred = mF * x;
		Eigen::MatrixXd PPred = mF * mP * mF.transpose() + mQ;

		// Update step
		Eigen::MatrixXd Ku = PPred * mH.transpose();
		Eigen::MatrixXd Kl = (mH * PPred * mH.transpose() + mR);
		Eigen::MatrixXd K = Ku * Kl.inverse();
		Eigen::VectorXd xNew = xPred + K * (mZ - mH * xPred);
		mP = PPred - K * mH * PPred;

		// Set the x and the z
		mX = xNew;
		mZ = mH * mX;
		
		// Save instance of PbD::PbdObject.
		mInstance = pObject;
	}
	
	bool KalmanFilter::isTimedOut(unsigned int threshold)
	{
	  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastUpdate).count() > threshold;
	}
	
	pbd_msgs::PbdObject KalmanFilter::getObject()
	{
	  // Write the pose maintained by the kalman-filter back into the object.
	  mInstance.poseEstimation.pose.position.x = mZ(0);
	  mInstance.poseEstimation.pose.position.y = mZ(1);
	  mInstance.poseEstimation.pose.position.z = mZ(2);
	  mInstance.poseEstimation.pose.orientation.w = mZ(3);
	  mInstance.poseEstimation.pose.orientation.x = mZ(4);
	  mInstance.poseEstimation.pose.orientation.y = mZ(5);
	  mInstance.poseEstimation.pose.orientation.z = mZ(6);
	  
	  // Return the instance updated by the kalman filter.
	  return mInstance;
	}
}
