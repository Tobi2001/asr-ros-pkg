#pragma once

// Package includes
#include <pl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <Pose.h>

#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "inference/model/foreground/ocm/shape/GaussianKernel.h"

namespace ProbabilisticSceneRecognition {

  /**
   * A gaussian mixture distribution maintaining a list of weighted gaussian kernels.
   */
  class GaussianMixtureDistribution {
  public:
    
    /**
    * Constructor.
    * 
    * @param pDimension The number of dimensions of the distribution.
    */
    GaussianMixtureDistribution(unsigned int pDimension);
    
    /**
    * Destructor.
    */
    ~GaussianMixtureDistribution();
    
    /**
     * Loads the working data from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     * @param pNode The name of the node that contains the distribution.
     */
    void load(boost::property_tree::ptree& pPt, std::string pNode);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param pVisualizer The visualizer for the secondary scene object represented by this node.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> pVisualizer);
    
    /**
     * Evaluates the gaussian mixture distribution for the given evidence and visualizes the result.
     * 
     * @param pVisualizer The visualizer for the secondary scene object represented by this node.
     * @param pPose The pose to evaluate the gaussian mixture distribution with.
     */
    void visualize(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> pVisualizer,
		   boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
    /**
     * Evaluate the gaussian mixture distribution with the given pose.
     * 
     * @param pPose The pose to evaluate the gaussian mixture distribution with.
     */
    double evaluate(boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
  private:
    
    /**
     * Converts the given pose into a 7D eigen Vector.
     * 
     * @param pPose The pose to convert to a 7d vector.
     * @return The 7d vector representing the pose.
     */
    Eigen::VectorXd getVectorFromObject(boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
  private:
    
    /**
     * The number of dimensions of the distribution.
     */
    unsigned int mDimension;
    
    /**
     * The gaussian kernels (and weights) forming the gaussian mixture distribution.
     */
    std::vector<GaussianKernel> mKernels;
  };
}