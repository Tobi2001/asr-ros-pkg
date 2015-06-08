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
   * A learner for the appearance term of the OCM.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class AppearanceTermLearner : public TermLearner {
  public:
    
    /**
     * Constructor.
     */
    AppearanceTermLearner();
    
    /**
     * Destructor.
     */
    ~AppearanceTermLearner();
    
    /**
     * Learns the term parameters.
     * 
     * @param pModel The OCM model that provides raw data and containers for the parameters to learn.
     */
    void learn(boost::shared_ptr<OcmModel> pModel);
    
  private:
    
    /**
     * Learns the mapping for the given node.
     * 
     * @param pModel The OCM model that provides raw data and containers for the parameters to learn.
     * @param pNode The node to learn the mapping for.
     */
    void learnMapping(boost::shared_ptr<OcmModel> pModel, boost::shared_ptr<OcmTree> pNode);
    
    /**
     * Learns the probability table for the given node.
     * 
     * @param pModel The OcmTree model that provides raw data and containers for the parameters to learn.
     * @param pNode The node to learn the probability table for.
     * @param pSlot Maps the given node to a row in the probability table.
     */
    void learnTable(boost::shared_ptr<OcmModel> pModel, boost::shared_ptr<OcmTree> pNode, unsigned int& pSlot);
  };
}