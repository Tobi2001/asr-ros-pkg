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
#include "inference/model/InferenceAlgorithm.h"

#include "inference/model/foreground/SceneObjectDescription.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract subclass of the also abstract InferenceAlgorithm class. It provides the calculations for a background scene object. This is basically a foreground scene object which is evaluated under the assumption of equal distribution.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ForegroundInferenceAlgorithm : public InferenceAlgorithm {
  public:

    /**
     * Constructor.
     * 
     * @param pSceneObjects The scene objects associated with this foreground scene.
     */
    ForegroundInferenceAlgorithm(boost::shared_ptr<std::vector<boost::shared_ptr<SceneObjectDescription> > >& pSceneObjects);
    
    /**
     * Destructor.
     */
    ~ForegroundInferenceAlgorithm();
    
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
     * The scene objects associated with this foreground scene.
     */
    boost::shared_ptr<std::vector<boost::shared_ptr<SceneObjectDescription> > > mSceneObjects;
    
  };
}