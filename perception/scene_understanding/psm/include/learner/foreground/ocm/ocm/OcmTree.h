#pragma once

// Global includes
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>

#include <trainer/TreeNode.h>
#include <trainer/source/ObjectSet.h>

#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>
#include <visualization/psm/ProbabilisticSecondarySceneObjectVisualization.h>

// Local includes
#include "learner/foreground/ocm/ocm/shape/GaussianMixtureModel.h"

#include "helper/MappedProbabilityTable.h"
#include "helper/ProbabilityTable.h"

#include "helper/MathHelper.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This tree represents the structure of the OCM. Every note equals a slot in the model and contains a set of parameters. These are filled by term learners. The tree provides a persistence mechanism.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class OcmTree {
  public:
    
    /**
     * Constructor.
     * 
     * @param pRoot Root node of the tree that represents the learned relationships.
     */
    OcmTree(const boost::shared_ptr<SceneModel::TreeNode> pRoot);
    
    /**
     * Destructor.
     */
    ~OcmTree();
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     * @param mParent The parent ocm node.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior, 
			      OcmTree* pParent);
    
    /**
     * Saves the shape information for the given node to XML.
     * 
     * @param pPt Data structure for handling XML operations.
     */
    void saveShape(boost::property_tree::ptree& pPt);
    
    /**
     * Returns the number of nodes in the tree.
     * 
     * @return The number of nodes in the tree.
     */
    unsigned int getNumberOfNodes();
    
  public:
    
    /**
     * The type of the object that is represented by this node.
     */
    std::string mType;
    
    /**
     * An object trajectory containing all observations of a single object over time.
     */
    boost::shared_ptr<SceneModel::ObjectSet> mObjectSet;
        
    /**
     * A list of child nodes.
     */
    std::vector<boost::shared_ptr<OcmTree> > mChildren;
    
    /**
     * The gaussian mixture model describing the shape of the node.
     * It states possible locations for the object assicated with this node.
     */
    GaussianMixtureModel mGaussianMixtureModelPosition;
    
    /**
     * The gaussian mixture model describing the shape of the node.
     * It states possible orientations for the object assicated with this node.
     */
    GaussianMixtureModel mGaussianMixtureModelOrientation;
    
    /**
     * Coordinates the secondary scene object visualizers.
     */
    boost::shared_ptr<Visualization::ProbabilisticSecondarySceneObjectVisualization> mVisualizer;
  };
}