#pragma once

// Global includes
#include <string>
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdSceneGraph.h>
#include <pbd_msgs/PbdNode.h>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

// Local includes
#include "learner/SceneLearner.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A learner for the background scene.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class BackgroundSceneLearner : public SceneLearner {    
  public:

    /**
     * Constructor.
     */
    BackgroundSceneLearner();
    
    /**
     * Destructor.
     */
    ~BackgroundSceneLearner();
    
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
     * The maximal number of objects existing in the world. Because we assume that there is only one instance per object class, this variable equals the number of all object classes. 
     */
    unsigned int mMaximalNumberOfObjects;
  };
}