#pragma once

// Global includes
#include <vector>

// Package includes
#include <pl.h>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "learner/foreground/ocm/ocm/shape/GaussianKernel.h"

namespace ProbabilisticSceneRecognition {
 
  /**
   * This class is a wrapper for a gaussian mixture model (GMM).
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class GaussianMixtureModel {
  public:

    /**
     * Constructor.
     */
    GaussianMixtureModel();
    
    /**
     * Destructor.
     */
    ~GaussianMixtureModel();
    
    /**
     * Adds a kernel to the distribution.
     * 
     * @param pKernel The kernel to add.
     */
    void addKernel(const GaussianKernel& pKernel);
    
    /**
     * Normalizes the weights to sum up to one. This is necessary after removing duplicate kernels.
     */
    void normalizeWeights();
    
    /**
     * Returns the number of kernels.
     * 
     * @return The number of kernels.
     */
    unsigned int getNumberOfKernels();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mSuperior);
    
    /**
     * Saves the content to XML.
     * 
     * @param pPt Datastructure for handling XML operations.
     * @param pNode The name of the XML node that should contain the distribution.
     */
    void save(boost::property_tree::ptree& pPt, std::string pNode);

  private:
    
    /**
     * The gaussian kernels the GMM is made of.
     */
    std::vector<GaussianKernel> mKernels;
  };
}