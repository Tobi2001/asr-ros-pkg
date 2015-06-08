#pragma once

// Global includes
#include <map>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// Package includes
#include <pl.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>

// Local includes
#include "helper/ProbabilityTable.h"

#include "inference/model/foreground/ocm/TermEvaluator.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * Subclass of the abstract TermEvaluator class responsible for the occlusion distribution which is part of P(h|theta).
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class OcclusionTermEvaluator : public TermEvaluator {
  public:

    /**
     * Constructor.
     */
    OcclusionTermEvaluator();
    
    /**
     * Destructor.
     */
    ~OcclusionTermEvaluator();
    
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
     * * @return Probability as determinded by the occlusion term.
     */
    double calculateProbabilityForHypothesis(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::vector<unsigned int> pAssignments);
    
    /**
     * Update the visualizers based on the evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     */
    void visualize(std::vector<pbd_msgs::PbdObject> pEvidenceList);
    
    /**
     * Returns the number of slots of the OCM (equals the number of distributions).
     * 
     * @return The number of slots of the OCM.
     */
    unsigned int getNumberOfSlots();
    
  private:
    
    /**
     * A probability table that describes the distribution of object types for every slot.
     * The table has two columns per slot. The first column is the probabilit for NOT OBSERVING the object, the second one for observing it.
     */
    boost::shared_ptr<ProbabilityTable> mTable;
  };
}