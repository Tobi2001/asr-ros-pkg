#pragma once

// Global includes
#include <vector>

// Package includes
#include <ros/ros.h>
#include <ros/console.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <pbd_msgs/PbdObject.h>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

// Local includes
#include "inference/model/ObjectEvidence.h"
#include "inference/model/SceneIdentifier.h"
#include "inference/model/SceneDescription.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Model for probabilistic scene recognition.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneModelDescription {    
  public:

    /**
     * Constructor.
     */
    SceneModelDescription();
    
    /**
     * Destructor.
     */
    ~SceneModelDescription();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPathToFile Path to the XML file that contains the modelin serialized form.
     * @param pAlgorithm The name of the inference algorithm that should be used.
     */
    void loadModelFromFile(std::string pPathToFile, std::string pAlgorithm);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior);
    
    /**
     * Integrated evidence abound objects found by the detection systems into the model.
     * Evidences are accumulated until an update is requested.
     * 
     * @param pObject PbDObject message containing data about the evidence.
     */
    void integrateEvidence(const boost::shared_ptr<const pbd_msgs::PbdObject>& pObject);
    
    /**
     * Integrate the learning data in form of a PbdSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void integrateSceneGraph(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph);
    
    /**
     * Update the model based on the accumulated evidence.
     */
    void updateModel();
    
    /**
     * Returns a list of all scenes containing their name and probability.
     * 
     * @param pSceneList The scene list including names and probabilities of all scenes.
     */
    void getSceneListWithProbabilities(std::vector<SceneIdentifier>& pSceneList);
    
  private:
    
    /**
     * An intelligent container for the object evidences. It states whether a given evidence is an update of an already known object or a new one.
     */
    ObjectEvidence mObjectEvidence;
    
    /**
     * Used for forwarding the evidences.
     * Put this here so we don't need to build a new one every time we got new evidence.
     */
    std::vector<pbd_msgs::PbdObject> mEvidenceList;
    
    /**
     * A list containing the background and foreground elements.
     */
    std::vector<boost::shared_ptr<SceneDescription> > mScenes;
  };
}