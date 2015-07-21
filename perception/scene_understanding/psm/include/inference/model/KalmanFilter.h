/*
 * KalmanFilter.h
 *
 *  Created on: 14.05.2014
 *      Author: schleich
 */

#pragma once

#include <chrono>

#include <Eigen/Dense>

#include <pbd_msgs/PbdObject.h>

namespace ProbabilisticSceneRecognition {
	/**
	 * The KalmanFilter class implements a KalmanFilter on a multidimensional space.
	 * A Kalman Filter estimates the real value vector in terms of the expected input error and expected output error.
	 * Which means the smaller the expected input error is, the more likely is the system to change the state.
	 * The bigger the expected output error is, the less likely the system is to change. And vice versa.
	 *
	 * @author Ralf Schleicher <mail@ralfschleicher.de>
	 */
	class KalmanFilter {
	private:
		/**
		 * If reset flag is true, reset the system to the current measurement.
		 */
		bool mReset;
		
		/**
		 * The time since the last update.
		 */
		std::chrono::high_resolution_clock::time_point lastUpdate;
		
		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mF;

		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mH;

		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mQ;

		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mR;

		/**
		 * A multidimensional matrix.
		 */
		Eigen::MatrixXd mP;

		/**
		 * A multidimensional vector.
		 */
		Eigen::VectorXd mX;

		/**
		 * A multidimensional vector.
		 */
		Eigen::VectorXd mZ;
		
		/**
		 * Instance of the current PbdObject.
		 */
		pbd_msgs::PbdObject mInstance;

	public:
		/**
		 * Creates a kalman filter for given input matrices.
		 * @param pObject The inintial measurement.
		 */
		KalmanFilter(pbd_msgs::PbdObject pObject);

		/**
		 * Destructor.
		 */
		~KalmanFilter();

		/**
		 * Resets the kalman filter.
		 */
		void reset();

		/**
		 * Updates the current state of the system
		 * @param pObject The new measurement to update the filter.
		 */
		void update(pbd_msgs::PbdObject pObject);
		
		/**
		 * Checks if the last update has been longer ago than the given threshold.
		 * 
		 * @param threshold Valid time in milliseconds since the last update.
		 */
		bool isTimedOut(unsigned int threshold);
		
		/**
		 * Returns the PbdObject wrapped by the filter.
		 * @return The PbdObject wrapped by the filter.
		 */
		pbd_msgs::PbdObject getObject();
	};
}