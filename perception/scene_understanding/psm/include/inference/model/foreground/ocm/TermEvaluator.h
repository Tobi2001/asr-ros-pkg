#pragma once

// Global includes
#include <vector>

// Package includes
#include <pl.h>

#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Abstract class to encapsulate the creation and maintenance of a joint probability distribution. The distributions are loaded from XML file. Object evidence will also be handles by this class, especially the case when new evidence was found and the model needs rebuilding.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class TermEvaluator {
  public:

    /**
     * Constructor.
     */
    TermEvaluator();
    
    /**
     * Destructor.
     */
    virtual ~TermEvaluator();
    
    /**
     * Loads the working data from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    virtual void load(boost::property_tree::ptree& pPt) = 0;
    
    /**
     * Integrate the learning data in form of a PbdSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    virtual void handleSceneGraph(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph) = 0;
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    virtual void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior) = 0;
    
    /**
     * Calculates the probability for a hypothesis with the given assignments.
     * 
     * @param pEvidenceList A list containing all evidences.
     * @param pAssignments Assignments of parts to slots.
     * @return Probability as determinded by the term wrapped here.
     */
    virtual double calculateProbabilityForHypothesis(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::vector<unsigned int> pAssignments) = 0;
    
    /**
     * Update the visualizers based on the evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     */
    virtual void visualize(std::vector<pbd_msgs::PbdObject> pEvidenceList) = 0;
    
    /**
     * Returns the number of slots of the OCM (equals the number of distributions).
     * 
     * @return The number of slots of the OCM.
     */
    virtual unsigned int getNumberOfSlots() = 0;
  };
}