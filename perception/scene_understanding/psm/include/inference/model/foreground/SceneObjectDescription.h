#pragma once

// Global includes
#include <string>
#include <vector>
#include <chrono>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>
#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>

// Local includes
#include "helper/MathHelper.h"

#include "inference/model/foreground/SceneObjectContent.h"

#include "inference/model/foreground/ocm/OcmSceneObjectContent.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class models a single instance of a scene object. A scene object contains information about the object as well as the scene. One could describe it as an object in the context of a scene.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneObjectDescription {    
  public:

    /**
     * Constructor.
     */
    SceneObjectDescription();

    /**
     * Destructor.
     */
    ~SceneObjectDescription();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
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
    
    /**
     * Returns the probability for the scene object modelled by this class.
     * 
     * @return Probability for this scene object.
     */
    double getSceneObjectProbability();
    
    /**
     * Returns the a priori probability of the scene object.
     * 
     * @return The a priori probability of the scene object.
     */
    double getSceneObjectPriori();
    
    /**
     * Returns the description of the scene object.
     * 
     * @return The description of the scene object.
     */
    std::string getDescription();
    
    /**
     * Marks the scene object with the best score.
     * 
     * @param pStatus True, to select the scene object as the one with the best score.
     */
    void setBestStatus(bool pStatus);
    
  private:
    
    /**
     * A priori probability of the scene object.
     */
    double mPriori;
    
    /**
     * The type of the content wrapped in this class.
     */
    std::string mType;
    
    /**
     * A short description of the scene object (e.g. Cup, Blue Plate, ...).
     */
    std::string mDescription;
    
    /**
     * A wrapper for the model that states how the scene object is modelled.
     */
    boost::shared_ptr<SceneObjectContent> mContent;
    
    /**
     * Coordinates the secondary scene object visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mVisualizer;
  };
}