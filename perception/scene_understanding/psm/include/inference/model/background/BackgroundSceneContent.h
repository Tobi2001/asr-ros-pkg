#pragma once

// Global includes
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>

// Local includes
#include "inference/model/SceneContent.h"

#include "inference/model/background/PowerSetBackgroundInferenceAlgorithm.h"
#include "inference/model/background/SummarizedBackgroundInferenceAlgorithm.h"
#include "inference/model/background/MultipliedBackgroundInferenceAlgorithm.h"
#include "inference/model/background/MaximumBackgroundInferenceAlgorithm.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This subclass of the SceneContent class is a wrapper for storing the resources required by a background scene. The background scene is based onthe same calculations as the foreground scene, but the information here is assumed equal distributed.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class BackgroundSceneContent : public SceneContent {
  public:
    
    /**
     * Constructor.
     */
    BackgroundSceneContent();
    
    /**
     * Destructor.
     */
    ~BackgroundSceneContent();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Initializes the inference algorithms. The algorithm that should be used is determined by the given string.
     * 
     * @param pAlgorithm The name of the inference algorithm that should be used.
     */
    void initializeInferenceAlgorithms(std::string pAlgorithm);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior);
    
    /**
     * Updates the model with new evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pRuntimeLogger A file handle for runtime logging.
     */
    void update(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger);
    
    /**
     * Integrate the learning data in form of a PbdSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph);
  };
}