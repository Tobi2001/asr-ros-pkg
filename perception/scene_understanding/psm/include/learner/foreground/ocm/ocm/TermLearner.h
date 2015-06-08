#pragma once

// Package includes
#include <boost/shared_ptr.hpp>

// Local includes
#include "learner/foreground/ocm/ocm/OcmModel.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * An abstract learner for the various terms of the OCM.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class TermLearner {
  public:
    
    /**
     * Constructor.
     */
    TermLearner();
    
    /**
     * Destructor.
     */
    virtual ~TermLearner();
    
    /**
     * Learns the term parameters.
     * 
     * @param pModel The OCM model that provides raw data and containers for the parameters to learn.
     */
   virtual void learn(boost::shared_ptr<OcmModel> pModel) = 0;
  };
}