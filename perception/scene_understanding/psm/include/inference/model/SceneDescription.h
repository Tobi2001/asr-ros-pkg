#pragma once

// Global includes
#include <vector>
#include <fstream>
#include <iostream>

// Package includes
#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>
#include <visualization/psm/ProbabilisticSceneVisualization.h>

// Local includes
#include "inference/model/SceneIdentifier.h"
#include "inference/model/SceneContent.h"

#include "inference/model/foreground/ForegroundSceneContent.h"	

#include "inference/model/background/BackgroundSceneContent.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class describes a scene. It's a wrapper for the for the scene content which contains the concrete realization of the scene.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneDescription {
  public:
    
    /**
     * Constructor.
     */
    SceneDescription();
    
    /**
     * Destructor.
     */
    ~SceneDescription();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     * @param pAlgorithm The name of the inference algorithm.
     */
    void load(boost::property_tree::ptree& pPt, std::string pAlgorithm);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior);
    
    /**
     * Updates the model with new evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     */
    void update(std::vector<pbd_msgs::PbdObject> pEvidenceList);
    
    /**
     * Integrate the learning data in form of a PbdSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph);
    
    /**
     * Calculates the probability of the scene.
     */
    void calculateSceneProbaility();
    
    /**
     * Returns the scene's metadata.
     * 
     * @return The metadata of the scene.
     */
    boost::shared_ptr<SceneIdentifier> getSceneIdentifier();
        
    /**
     * Sets the scene content of this scene.
     * 
     * @param pIdentifier The identifier holding the scenes metadata.
     */
    void setSceneIdentifier(boost::shared_ptr<SceneIdentifier> pIdentifier);
    
  private:
    
    /**
     * handle for the file that will contain the results from the runtime test.
     */
    std::ofstream mRuntimeFile;
    
    /**
     * A wrapper for the scene's metadata.
     */
    boost::shared_ptr<SceneIdentifier> mIdentifier;
    
    /**
     * A container holding the resources required by the scene.
     * A background scene will hold other resources than a foreground scene.
     */
    boost::shared_ptr<SceneContent> mContent;
    
    /**
     * Coordinates the primary scene object visualization.
     */
    boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mVisualizer;
  };
}