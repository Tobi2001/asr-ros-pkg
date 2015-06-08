#pragma once

// Global includes
#include <string>
#include <vector>

// Package includes
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract class for an inference algorithm. In this case inference is the process of determining the scene probability based on the scene model.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class InferenceAlgorithm {
  public:

    /**
     * Constructor.
     */
    InferenceAlgorithm();
    
    /**
     * Destructor.
     */
    virtual ~InferenceAlgorithm();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    virtual void load(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Executes the inference based on the given evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    virtual void doInference(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger) = 0;
    
    /**
     * Returns the probability calculated by the inference process.
     * 
     * @return Probability for this scene.
     */
    virtual double getProbability() = 0;
  };
}