#pragma once

// Global includes
#include <vector>

// Package includes
#include <ros/ros.h>
#include <ros/console.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

// Local includes
#include "learner/SceneLearner.h"

#include "learner/background/BackgroundSceneLearner.h"

#include "learner/foreground/ForegroundSceneLearner.h"

#include "learner/foreground/ocm/OcmForegroundSceneLearner.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Defines the type for the OCM based foreground scene model.
   */
  static const std::string OCM_SCENE_MODEL = "ocm";
  
  /**
   * Learner for the probabilistic scene model.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneModelLearner {    
  public:

    /**
     * Constructor.
     * 
     * @param pForegroundSceneModel The type of model that should be used for foreground scene representation.
     * @param pWorkspaceVolume The volume of the workspace in cubic meters.
     * @param pStaticBreakRatio The maximum ration the relationship between two objects may break.
     * @param pTogetherRatio The minimum ratio that two objects must be together.
     * @param pMaxAngleDeviation Maximum angle deviation between two objects before counting as break.
     */
    SceneModelLearner(std::string pForegroundSceneModel, double pWorkspaceVolume, double pStaticBreakRatio, double pTogetherRatio, double pMaxAngleDeviation);
    
    /**
     * Destructor.
     */
    ~SceneModelLearner();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior);
    
    /**
     * Adds a PbdSceneGraph message to the learner.
     * 
     * @param pExample PbdSceneGraph message containing an example for a scene.
     */
    void addExample(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pExample);
    
    /**
     * Calculates the model parameters based on the collected evidence.
     */
    void generateSceneModel();
    
    /**
     * Saves the model to an XML file.
     * 
     * @param pPathToFile The CMl file the scene model should be written to.
     */
    void saveSceneModelToFile(std::string pPathToFile);
    
  private:
    
    /**
     * The type of model that should be used for foreground scene representation.
     */
    std::string mForegroundSceneModel;
    
    /**
     * The volume of the workspace the scene takes place in qubic meters.
     */
    double mWorkspaceVolume;
    
    /**
     * Parameters of heuristics used for hierarchical clustering.
     */
    double mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation;
    
    /**
     * A seperate learner for the background model.
     */
    BackgroundSceneLearner mBackgroundSceneLearner;
    
    /**
     * A list of all foreground scene learners. If an PbdSceneGraph message could not be associated with a learner in the list, a new scene learner is automatically added.
     */
    std::vector<boost::shared_ptr<ForegroundSceneLearner> > mSceneLearners;
  };
}