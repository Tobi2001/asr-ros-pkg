#pragma once

// Package includes
#include <pl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "helper/SerializationHelper.h"

namespace ProbabilisticSceneRecognition {

  /**
   * This class is a wrapper for a single gaussian kernel.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class GaussianKernel {
  public:

    /**
     * Constructor.
     */
    GaussianKernel();
    
    /**
     * Destructor.
     */
    ~GaussianKernel();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mSuperior);
    
    /**
     * Compares this kernel with the given one.
     * 
     * @param pKernel The kernel to compare this one with.
     * @return True, if both kernels are equal.
     */
    bool compare(const GaussianKernel pKernel);
    
    /**
     * Saves the content to XML.
     * 
     * @param pPt Datastructure for handling XML operations.
     */
    void save(boost::property_tree::ptree& pPt);
    
  public:
    
    /**
     * The weight of the gaussian kernel.
     */
    double mWeight;
    
    /**
     * The mean vector of the gaussien kernel.
     */
    boost::shared_ptr<Eigen::VectorXd> mMean;
    
    /**
     * The cocovariance matrix of the gaussian kernel.
     */
    boost::shared_ptr<Eigen::MatrixXd> mCovariance;
  };
}