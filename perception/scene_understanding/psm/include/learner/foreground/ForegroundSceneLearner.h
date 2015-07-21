#pragma once

// Global includes
#include <string>
#include <vector>

// Package includes
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

// Local includes
#include "learner/SceneLearner.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract learner for a foreground scene.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ForegroundSceneLearner : public SceneLearner {    
  public:

    /**
     * Constructor.
     * 
     * @param pExample A PbdSceneGraph message containing object observation trajectories.
     */
    ForegroundSceneLearner(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pExample);
    
    /**
     * Destructor.
     */
    virtual ~ForegroundSceneLearner();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    virtual void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior) = 0;
    
    /**
     * Saves the scene to XML file.
     * 
     * @param pPt Datastructure for handling XML operations.
     */
    virtual void save(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Learns the scene.
     */
    virtual void learn() = 0;
    
    /**
     * Parameters for the clustering algorithm.
     * @param pStaticBreakRatio The maximum ration the relationship between two objects may break.
     * @param pTogetherRatio The minimum ratio that two objects must be together.
     * @param pMaxAngleDeviation Maximum angle deviation between two objects before counting as break.
     */
    void setClusteringParameters(double pStaticBreakRatio, double pTogetherRatio, double pMaxAngleDeviation);
    
  protected:
    
    /**
     * Parameters of heuristics used for hierarchical clustering.
     */
    double mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation;
  };
}