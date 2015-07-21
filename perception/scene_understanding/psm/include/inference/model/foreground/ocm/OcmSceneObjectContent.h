#pragma once

// Global includes
#include <string>
#include <vector>
#include <sstream>

// Package includes
#include <pl.h>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pbd_msgs/PbdObject.h>
#include <pbd_msgs/PbdSceneGraph.h>

#include <visualization/psm/ProbabilisticPrimarySceneObjectVisualization.h>

// Local includes
#include "inference/model/foreground/SceneObjectContent.h"

#include "inference/model/foreground/ocm/TermEvaluator.h"

#include "inference/model/foreground/ocm/shape/ShapeTermEvaluator.h"

#include "inference/model/foreground/ocm/appearance/AppearanceTermEvaluator.h"

#include "inference/model/foreground/ocm/occlusion/OcclusionTermEvaluator.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class is a container for the data used by a scene object which is based on the Object Constellation Model (OCM).
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class OcmSceneObjectContent : public SceneObjectContent {
  public:

    /**
     * Constructor.
     */
    OcmSceneObjectContent();
    
    /**
     * Destructor.
     */
    ~OcmSceneObjectContent();
    
    /**
     * Loads the model from an XML file.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Initializes the visualization mechanism.
     * 
     * @param mSuperior The superior visualizer coordinating the scene visualizers.
     */
    void initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mSuperior);
    
    /**
     * Updates the model with new evidence.
     * 
     * @param pEvidenceList A list containing all evidences.
     */
    void update(std::vector<pbd_msgs::PbdObject> pEvidenceList);
    
    /**
     * Integrate the learning data in form of a PbdSceneGraph into the model.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
     */
    void update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph);
    
    /**
     * Returns the probability for the scene object modelled by this class.
     * 
     * @return Probability for this scene object.
     */
    double getSceneObjectProbability();
    
    /**
     * Marks the scene object with the best score.
     * 
     * @param pStatus True, to select the scene object as the one with the best score.
     */
    void setBestStatus(bool pStatus);
    
  private:
    
    /**
     * The probability for this scene object based on the evidence.
     */
    double mProbability;
    
    /**
     * The number of slots of the OCM.
     */
    unsigned int mNumberOfSlots;
    
    /**
     * The term evaluators responsible for generation of the distributions that are part of the OCM.
     */
    std::vector<boost::shared_ptr<TermEvaluator> > mEvaluators;
    
    /**
     * Pointer to the visualizer.
     */
    boost::shared_ptr<Visualization::ProbabilisticPrimarySceneObjectVisualization> mVisualizer;
  };
}