#pragma once

// Global includes
#include <string>
#include <vector>

// Package includes
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>

// Local includes
#include "inference/model/foreground/ForegroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Implementation of the abstract ForegroundInferenceAlgorithm class. It calculates and evaluates the power set of all foreground scene objects and summarized the result of each subset.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class PowerSetForegroundInferenceAlgorithm : public ForegroundInferenceAlgorithm {
  public:
    
    /**
     * Constructor.
     * 
     * @param pSceneObjects The scene objects associated with this foreground scene.
     */
    PowerSetForegroundInferenceAlgorithm(boost::shared_ptr<std::vector<boost::shared_ptr<SceneObjectDescription> > >& pSceneObjects);
    
    /**
     * Destructor.
     */
    ~PowerSetForegroundInferenceAlgorithm();
    
    /**
     * Executes the inference based on the given evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    void doInference(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger);
    
    /**
     * Returns the probability calculated by the inference process.
     * 
     * @return Probability for this scene.
     */
    double getProbability();
    
  private:
    
    /**
     * The probability calculated by this algorithm.
     */
    double mProbability;
  };
}