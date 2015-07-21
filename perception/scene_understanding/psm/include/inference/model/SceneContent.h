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
   * Abstract class for modelling a specific type of scene (e.g. fore- or background scene).
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneContent {
  public:

    /**
     * Constructor.
     */
    SceneContent();
    
    /**
     * Destructor.
     */
    virtual ~SceneContent();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    virtual void load(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Initializes the inference algorithms. The algorithm that should be used is determined by the given string.
     * 
     * @param pAlgorithm The name of the inference algorithm that should be used.
     */
    virtual void initializeInferenceAlgorithms(std::string pAlgorithm) = 0;
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    virtual void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior) = 0;
    
    /**
     * Updates the model with new evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    virtual void update(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger) = 0;
    
    /**
     * Integrate the learning data in form of a PbdSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    virtual void update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph) = 0;
    
    /**
     * Returns the probability for the scene modelled by this class.
     * 
     * @return Probability for this scene.
     */
    double getSceneProbability();
    
  protected:
    
    /**
     * Sets the inference algorithm.
     * 
     * @param pAlgorithm The algorithm used to infer the scene probability.
     */
    void setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm> pAlgorithm);
    
    /**
     * Loads the data used by the inference algorithm from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void loadInferenceAlgorithm(boost::property_tree::ptree& pPt);
    
    /**
     * Executes the inference process.
     * 
     * @param pEvidenceList The evidence found.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    void doInference(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger);
    
  private:
    
    /**
     * The algorithm used for inference.
     */
    boost::shared_ptr<InferenceAlgorithm> mAlgorithm;
    
  };
}