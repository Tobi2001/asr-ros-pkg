#pragma once

// Global includes
#include <string>
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>
#include <visualization/psm/ProbabilisticSceneVisualization.h>

#include <trainer/PSMTrainer.h>

// Local includes
#include "learner/foreground/ForegroundSceneLearner.h"

#include "learner/foreground/ocm/SceneObjectLearner.h"

#include "learner/foreground/ocm/ocm/OcmSceneObjectLearner.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A learner for a foreground scene based on the OCM.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class OcmForegroundSceneLearner : public ForegroundSceneLearner {    
  public:

    /**
     * Constructor.
     * 
     * @param pExample A PbdSceneGraph message containing object observation trajectories.
     */
    OcmForegroundSceneLearner(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pExample);
    
    /**
     * Destructor.
     */
    ~OcmForegroundSceneLearner();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior);
    
    /**
     * Saves the scene to XML file.
     * 
     * @param pPt Datastructure for handling XML operations.
     */
    void save(boost::property_tree::ptree& pPt);
    
    /**
     * Learns the scene.
     */
    void learn();
    
  private:
    
    /**
     * A list of all scene object learners.
     */
    std::vector<boost::shared_ptr<SceneObjectLearner> > mSceneObjectLearners;
    
    /**
     * Coordinates the primary scene object visualization.
     */
    boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mVisualizer;
  };
}