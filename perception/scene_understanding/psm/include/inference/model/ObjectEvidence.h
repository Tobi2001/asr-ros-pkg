#pragma once

// Global includes
#include <map>
#include <string>
#include <vector>

// Local includes
#include <ros/ros.h>
#include <boost/foreach.hpp>

// Package includes
#include <pbd_msgs/PbdObject.h>

#include "inference/model/KalmanFilter.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class acts as a container for the object evidence. It states whether a given evidence is an update of an already known object or a new one. It is possible to return the evidences as a vector that can be distributed to the parts of the model that require the evicendes.
   * 
   * The container is able to forgot already seen objects if the specified threshold is exceeded.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ObjectEvidence {
  public:

    /**
     * Constructor.
     */
    ObjectEvidence();
    
    /**
     * Destructor.
     */
    ~ObjectEvidence();
    
    /**
     * Adds an object evidence to the buffer.
     * 
     * @param pObject PbDObject message containing information about the evidence.
     */
    void push(const boost::shared_ptr<const pbd_msgs::PbdObject>& pObject);
    
    /**
     * Checks, if new evidence has been added since the last model update.
     * 
     * @return True, if new evidence was added and not yet integrated into the model.
     */
    bool hasWaitingEvidences();
    
    /**
     * Integrates all evidences into the list and checks whether there are new evidences found.
     * This method must be called before the evidence list can be requested.
     */
    void update();
    
    /**
     * Returns a list of all evidences found till now.
     * 
     * @param pEvidences A vector containing all evidences.
     */
    void getEvidences(std::vector<pbd_msgs::PbdObject>& pEvidences);
    
  private:
    
    /**
     * Evidences older than this time in milliseconds are erased from the list.
     */
    int mTimeout;
    
    /**
     * A temporary buffer for accumulating evidences.
     */
    std::vector<pbd_msgs::PbdObject> mBuffer;
    
    /**
     * A map for storing the object evidences, indexed by the object type and identifier.
     * The tuple (object type, object identifier) is an unique key for every found object.
     */
    std::map<std::string, std::map<std::string, KalmanFilter> > mObjectEvidences;
  };
}