#pragma once

// Global includes
#include <string>
#include <vector>

// Package includes
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>

#include <trainer/PSMTrainer.h>

namespace ProbabilisticSceneRecognition {
  
  /**
   * An abstract learner for a scene object.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneObjectLearner {    
  public:

    /**
     * Constructor.
     * 
     * @param pSceneObjectType The type of the scene object.
     */
    SceneObjectLearner(std::string pSceneObjectType);
    
    /**
     * Destructor.
     */
    virtual ~SceneObjectLearner();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    virtual void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior) = 0;
    
    /**
     * Saves the scene to XML file.
     * 
     * @param pPt Datastructure for handling XML operations.
     */
    virtual void save(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Learns the scene object.
     * 
     * @param mExamplesList A list of all examples for the scene this scene object belongs to.
     */
    virtual void learn(std::vector<boost::shared_ptr<const pbd_msgs::PbdSceneGraph> > mExamplesList,
      boost::shared_ptr<SceneModel::TreeNode> tree) = 0;
   
    /**
      * Checks, if this scene object has the given type.
      * 
      * @param pSceneObjectType The type to check for.
      */
    bool hasType(std::string pSceneObjectType);
    
    /**
     * Parameters for the clustering algorithm.
     * @param pStaticBreakRatio The maximum ration the relationship between two objects may break.
     * @param pTogetherRatio The minimum ratio that two objects must be together.
     * @param pMaxAngleDeviation Maximum angle deviation between two objects before counting as break.
     */
    void setClusteringParameters(double pStaticBreakRatio, double pTogetherRatio, double pMaxAngleDeviation);
    
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
     * The type of the scene object. Because we don't use any instance information this is also the identity of the scene object.
     */
    std::string mSceneObjectType;
    
    /**
     * The volume of the workspace the scene takes place in qubic meters.
     */
    double mWorkspaceVolume;
    
    /**
     * Parameters of heuristics used for hierarchical clustering.
     */
    double mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation;
  };
}