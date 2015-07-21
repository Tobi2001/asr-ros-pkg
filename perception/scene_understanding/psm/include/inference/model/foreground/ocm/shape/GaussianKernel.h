#pragma once

// Global includes
#include <cmath>
#include <vector>
#include <random>

// Package includes
#include <pl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pbd_msgs/PbdObject.h>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "helper/SerializationHelper.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class implements a single 6D gaussian kernel. It stores a weight, mean vector and covariance matrix.
   */
  class GaussianKernel {
  private:
    
    /**
     * The number of samples used for monte carlo integration of the gaussian.
     */
    static const int MC_SAMPLES = 100;
  public:

    /**
    * Constructor.
    * 
    * @param pDimension The number of dimensions of the kernel.
    * @param pPt Data structure for performing XML operations.
    */
    GaussianKernel(unsigned int pDimension, boost::property_tree::ptree& pPt);
    
    /**
    * Destructor.
    */
    ~GaussianKernel();
    
    /**
     * Loads the working data from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mVisualizer The visualizer for the secondary scene object represented by this node.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer);
    
    /**
     * Evaluates the gaussian kernel for the given evidence and visualizes the result.
     * 
     * @param mVisualizer The visualizer for the secondary scene object represented by this node.
     * @param pEvidence The evidence for that the visualization should take place.
     */
    void visualize(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer,
		   Eigen::VectorXd pEvidence);
    
    /**
     * Evaluate the gaussian kernel with the given evidence.
     * 
     * @param pEvidence The evidence to evaluate the kernel with.
     * @return Probability for the given evidence.
     */
    double evaluate(Eigen::VectorXd pEvidence);
    
    /**
     * Return the weight of the gaussian kernel.
     */
    double getWeight();
    
  private:
    
    /**
     * The number of dimensions of the distribution.
     */
    unsigned int mDimension;
    
    /**
     * The weight of the gaussian kernel determining how string it contributes to the gaussian mixture distribution.
     */
    double mWeight;
    
    /**
     * The mean vector of the multivariate gaussian.
     */
    boost::shared_ptr<Eigen::VectorXd> mMean;
    
    /**
     * The covariance matrix of the multivariate gaussian.
     */
    boost::shared_ptr<Eigen::MatrixXd> mCovariance;
    
    /**
     * The position part of the mean vector.
     */
    boost::shared_ptr<Eigen::Vector3d> mPositionMean;
    
    /**
     * The position part of the covariance matrix.
     */
    boost::shared_ptr<Eigen::Matrix3d> mPositionCovariance;
  };
}