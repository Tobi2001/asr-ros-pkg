#pragma once

// Global includes
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

// Package includes
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>

// Local includes
#include "inference/model/InferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract subclass of the also abstract InferenceAlgorithm class. It provides the calculations for a background scene object. This is basically a foreground scene object which is evaluated under the assumption of equal distribution.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class BackgroundInferenceAlgorithm : public InferenceAlgorithm {
  public:

    /**
     * Constructor.
     */
    BackgroundInferenceAlgorithm();
    
    /**
     * Destructor.
     */
    ~BackgroundInferenceAlgorithm();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
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
    
  protected:
    
    /**
     * Calculates the probability of a single background scene object.
     * There are multiple background scene objects, but they all have the same probability.
     * 
     * @param pNumberOfEvidence The number of objects to consider for the background calculation.
     * @param pNumberOfSlots The total number of slots.
     * @return The probability of a single background scene object.
     */
    double calculateProbabilityOfBackgroundSceneObject(unsigned int pNumberOfEvidence, unsigned int pNumberOfSlots);
    
  private:
    
    /**
     * The number of all possible object instances in the/our world.
     */
    unsigned int mNumberOfObjectClasses;
    
    /**
     * The volume of the space we're operating in.
     */
    double mVolumeOfWorkspace;
    
  };
}