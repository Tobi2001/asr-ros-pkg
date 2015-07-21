#pragma once

// Global includes
#include <vector>

// Package includes
#include <pl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <Pose.h>

#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>

// Local includes
#include "inference/model/foreground/ocm/shape/HierarchicalShapeModelNode.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class encapsulates the functionalities and data structures required by the hierarchical shape model.
   * It also acts as the root node of the tree.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class HierarchicalShapeModel {
  public:

    /**
     * Constructor.
     */
    HierarchicalShapeModel();
    
    /**
     * Destructor.
     */
    ~HierarchicalShapeModel();
    
    /**
     * Loads the working data from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Integrate the learning data in form of a PbdSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void handleSceneGraph(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior);
    
    /**
     * Calculates the probability for a hypothesis with the given assignments.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pAssignments Assignments of parts to slots.
     * @return Probability as determined by the hierarchical shape model.
     */
    double calculateProbabilityForHypothesis(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::vector<unsigned int> pAssignments);
    
    /**
     * Update the visualizers based on the evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     */
    void visualize(std::vector<pbd_msgs::PbdObject> pEvidenceList);
    
    /**
     * Return the number of nodes in the OCM.
     */
    unsigned int getNumberOfNodes();
    
  private:
    
    /**
     * The volume of the workspace in cubic meters.
     */
    double mWorkspaceVolume;
    
    /**
     * The pose of the root object in absolute coordinates. This variable is used during inference and reused in the visualization.
     */
    boost::shared_ptr<ResourcesForPsm::Pose> mAbsolutePose;
    
    /**
     * The chrildren of this node.
     */
    std::vector<HierarchicalShapeModelNode> mChildren;
    
    /**
     * A copy of the visualizer that coordinates the secondary scene object visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mVisualizer;
  };
}