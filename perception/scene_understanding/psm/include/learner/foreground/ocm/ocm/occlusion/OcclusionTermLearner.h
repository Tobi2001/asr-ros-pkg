#pragma once

// Package includes
#include <boost/shared_ptr.hpp>

#include <trainer/source/Object.h>

// Local includes
#include "learner/foreground/ocm/ocm/OcmTree.h"
#include "learner/foreground/ocm/ocm/OcmModel.h"
#include "learner/foreground/ocm/ocm/TermLearner.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A learner for the occlusion term of the OCM.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class OcclusionTermLearner : public TermLearner {
  public:
    
    /**
     * Constructor.
     */
    OcclusionTermLearner();
    
    /**
     * Destructor.
     */
    ~OcclusionTermLearner();
    
    /**
     * Learns the term parameters.
     * 
     * @param pModel The OCM model that provides raw data and containers for the parameters to learn.
     */
    void learn(boost::shared_ptr<OcmModel> pModel);
  };
}