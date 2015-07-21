#pragma once

// Global includes
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract class for modelling the contents of a scene object.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneObjectContent {
  public:

    /**
     * Constructor.
     */
    SceneObjectContent();
    
    /**
     * Destructor.
     */
    virtual ~SceneObjectContent();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    virtual void load(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    virtual void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior) = 0;
    
    /**
     * Updates the model with new evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     */
    virtual void update(std::vector<pbd_msgs::PbdObject> pEvidenceList) = 0;
    
    /**
     * Integrate the learning data in form of a PbdSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    virtual void update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph) = 0;
    
    /**
     * Returns the probability for the scene object modelled by this class.
     * 
     * @return Probability for this scene object.
     */
    virtual double getSceneObjectProbability() = 0;
    
    /**
     * Marks the scene object with the best score.
     * 
     * @param pStatus True, to select the scene object as the one with the best score.
     */
    virtual void setBestStatus(bool pStatus) = 0;
  };
}