#pragma once

// Global includes
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <trainer/TreeNode.h>

#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticSceneVisualization.h>
#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>

// Local includes
#include "learner/foreground/ocm/SceneObjectLearner.h"

#include "learner/foreground/ocm/ocm/OcmModel.h"
#include "learner/foreground/ocm/ocm/TermLearner.h"

#include "learner/foreground/ocm/ocm/shape/ShapeTermLearner.h"

#include "learner/foreground/ocm/ocm/appearance/AppearanceTermLearner.h"

#include "learner/foreground/ocm/ocm/occlusion/OcclusionTermLearner.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A learner for a scene object based on the OCM.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class OcmSceneObjectLearner : public SceneObjectLearner {
  public:

    /**
     * Constructor.
     * 
     * @param pSceneObjectType The type of the scene object.
     */
    OcmSceneObjectLearner(std::string pSceneObjectType);
    
    /**
     * Destructor.
     */
    ~OcmSceneObjectLearner();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneVisualization> mSuperior);
    
    /**
     * Saves the scene to XML file.
     * 
     * @param pPt Datastructure for handling XML operations.
     */
    void save(boost::property_tree::ptree& pPt);
    
    /**
     * Learns the scene object.
     * 
     * @param pExamplesList A list of all examples for the scene this scene object belongs to.
     * @param pTree The relation tree.
     */
   void learn(std::vector<boost::shared_ptr<const pbd_msgs::PbdSceneGraph> > pExamplesList,
     boost::shared_ptr<SceneModel::TreeNode> pTree);
   
  private:
    
    /**
     * The root node of the OCM tree required for learning the parameters.
     */
    boost::shared_ptr<OcmModel> mOcmModel;
    
    /**
     * A list of term learners. They calculate the parameter for the terms the OCM consists of.
     */
    std::vector<boost::shared_ptr<TermLearner> > mTermLearners;
    
    /**
     * Coordinates the secondary scene object visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mVisualizer;
  };
}