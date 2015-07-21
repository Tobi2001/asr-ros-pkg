#pragma once

// Package includes
#include <boost/shared_ptr.hpp>

#include <Pose.h>

// Local includes
#include "learner/foreground/ocm/ocm/OcmTree.h"
#include "learner/foreground/ocm/ocm/OcmModel.h"
#include "learner/foreground/ocm/ocm/TermLearner.h"

#include "learner/foreground/ocm/ocm/shape/GMMParameterEstimator.h"

#include "helper/MathHelper.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A learner for the shape term of the OCM.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ShapeTermLearner : public TermLearner {
  public:
    
    /**
     * Constructor.
     */
    ShapeTermLearner();
    
    /**
     * Destructor.
     */
    ~ShapeTermLearner();
    
    /**
     * Learns the term parameters.
     * 
     * @param pModel The OCM model that provides raw data and containers for the parameters to learn.
     */
   void learn(boost::shared_ptr<OcmModel> pModel);
   
  private:
    
    /**
     * Learns the term parameters.
     * 
     * @param pNode OCM parent node that provides raw data and containers for the parameters to learn.
     */
   void learn(boost::shared_ptr<OcmTree> pNode);
    
    /**
     * Learns the shape term parameters for a parent/child pair.
     * 
     * @param pParent The parent node the pose should be learned relative to.
     * @param pChild The child node to assign the learned pose to.
     */
    void learnNodePose(boost::shared_ptr<OcmTree> pParent, boost::shared_ptr<OcmTree> pChild);
    
  private:
    
    /**
     * The minmal and maximal number of kernels.
     */
    int mNumberKernelsMin, mNumberKernelsMax;
    
    /**
     * The number of runs per kernel.
     */
    int mRunsPerKernel;
    
    /**
     * For every sample, n noised samples are generated to make orientation and position learning more tolerant.
     */
    int mNumberOfSyntheticSamples;
    
    /**
     * The intervals (position and orientation) for the sample relaxiation.
     */
    double mIntervalPosition, mIntervalOrientation;
    
    /**
     * Path to the orientation plots.
     */
    std::string mPathOrientationPlots;
    
    /**
     * Interface to the private namespace of the ros node.
     */
    ros::NodeHandle mPrivateNamespaceHandle;
  };
}