#pragma once

// Global includes
#include <string>
#include <vector>

// Package includes
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneModelVisualization.h>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract learner for a single scene.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneLearner {    
  public:

    /**
     * Constructor.
     * 
     * @param pSceneName The scene of the scene.
     */
    SceneLearner(std::string pSceneName);
    
    /**
     * Destructor.
     */
    virtual ~SceneLearner();
    
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
     * Checks, if the given PbdSceneGraph message contains an example for this scene.
     * 
     * @return True, if the message contains an example for this scene.
     */
    bool isExampleForScene(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pExample);
    
    /**
     * Adds a PbdSceneGraph message to the learner.
     * 
     * @param pExample PbdSceneGraph message containing an example for the given scene.
     */
    void addExampleToScene(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pExample);
    
    /**
     * Sets the volume of the workspace.
     * 
     * @param pValue The volume of the workspace.
     */
    void setVolumeOfWorkspace(double pValue);
    
    /**
     * Sets the a priori probability of the scene.
     * 
     * @param pPriori The a priori probability of the scene.
     */
    void setPriori(double pPriori);
    
  protected:
    
    /**
     * The a priori probability for this scene.
     * Will be set to an equal distribution for all scenes.
     */
    double mPriori;
    
    /**
     * The volume of the workspace the scene takes place in qubic meters.
     */
    double mWorkspaceVolume;
    
    /**
     * The name of the scene. It is required for filtering PbdSceneGraph messages and the export to file.
     */
    std::string mSceneName;
    
    /**
     * A list of all examples provided for this scene.
     */
    std::vector<boost::shared_ptr<const pbd_msgs::PbdSceneGraph> > mExamplesList;
  };
}