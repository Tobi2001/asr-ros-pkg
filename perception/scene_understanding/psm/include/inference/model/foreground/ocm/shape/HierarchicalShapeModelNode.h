#pragma once

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
#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "inference/model/foreground/ocm/shape/GaussianMixtureDistribution.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A node in object constellation model (OCM). It is responsible for maintaining the gaussian mixture distributions for this node. Note that all distributions are modelled in the coordinate system of their parent nodes and the transformation of evidence into this coordinate system is also handled here.
   */
  class HierarchicalShapeModelNode {
  public:

    /**
    * Constructor.
    * 
    * @param pPt Data structure for performing XML operations.
    */
    HierarchicalShapeModelNode(boost::property_tree::ptree& pPt);
    
    /**
    * Destructor.
    */
    ~HierarchicalShapeModelNode();
    
    /**
     * Loads the working data from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Integrate the learning data in form of a PbdSceneGraph into the model.
     *
     * @param pParent The PbdNode that acts as parent for this node.
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void handleSceneGraph(pbd_msgs::PbdNode& pParent, const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior);
    
    /**
     * Sets the absolute pose of the parent node.
     * 
     * @param pPose The absolute pose.
     */
    void setAbsoluteParentPose(boost::shared_ptr<ResourcesForPsm::Pose> pPose);
    
    /**
     * Calculates the probability for a hypothesis with the given assignments.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pAssignments Assignments of parts to slots.
     * @param pSlotId The id of the slot this node is associated with.
     * @param pCut True, if a zero-object was assigned to a direct parent node.
     * @return Probability as determined by this subtree the hierarchical shape model.
     */
    double calculateProbabilityForHypothesis(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::vector<unsigned int> pAssignments, unsigned int& pSlotId, bool pCut);
    
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
     * The type of the object that is represented by this node.
     */
    std::string mSceneObject;
    
    /**
     * The pose of this object in absolute coordinates. This variable is used during inference and reused in the visualization.
     */
    boost::shared_ptr<ResourcesForPsm::Pose> mAbsolutePose;
    
    /**
     * The pose of this object in coordinates relative to the parent frame. This variable is used during inference and reused in the visualization.
     */
    boost::shared_ptr<ResourcesForPsm::Pose> mRelativePose;
    
    /**
     * The pose of the parent object in absolute coordinates.
     */
    boost::shared_ptr<ResourcesForPsm::Pose> mAbsoluteParentPose;
    
    /**
     * The gaussian mixture distribution representing the position part of the shape.
     */
    boost::shared_ptr<GaussianMixtureDistribution> mGaussianMixtureDistributionPosition;
    
    /**
     * The gaussian mixture distribution representing the orientation part of the shape.
     */
    boost::shared_ptr<GaussianMixtureDistribution> mGaussianMixtureDistributionOrientation;
    
    /**
     * A list of all samples that were used to learn the gaussian mixture distributions.
     */
    std::vector<Eigen::Vector3d> mRawData;
    
    /**
     * The chrildren of this node.
     */
    std::vector<HierarchicalShapeModelNode> mChildren;
    
    /**
     * Coordinates the secondary scene object visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer;
  };
}