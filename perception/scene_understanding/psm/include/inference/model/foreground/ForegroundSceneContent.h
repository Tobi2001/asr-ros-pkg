#pragma once

// Global includes
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

#include "inference/model/foreground/PowerSetForegroundInferenceAlgorithm.h"
#include "inference/model/foreground/SummarizedForegroundInferenceAlgorithm.h"
#include "inference/model/foreground/MultipliedForegroundInferenceAlgorithm.h"
#include "inference/model/foreground/MaximumForegroundInferenceAlgorithm.h"

#include "inference/model/foreground/SceneObjectDescription.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This subclass of SceneContent class represents a foreground scene. A foreground scene is a scene that contains a model for describing object relations.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ForegroundSceneContent : public SceneContent {
  public:
    
    /**
     * Constructor.
     */
    ForegroundSceneContent();
    
    /**
     * Destructor.
     */
    ~ForegroundSceneContent();
    
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
    
  protected:
    
    /**
     * The scene objects associated with this foreground scene.
     */
    boost::shared_ptr<std::vector<boost::shared_ptr<SceneObjectDescription> > > mSceneObjects;
  };
}