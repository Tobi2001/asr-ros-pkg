#pragma once

// Global includes
#include <cmath>
#include <vector>

// Package includes
#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

#include <pbd_msgs/PbdObject.h>

#include <tf/transform_listener.h>

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class transforms PbdObjects into a destination frame.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ObjectTransformation {
  public:
    
    /**
     * Constructor.
     */
    ObjectTransformation();
    
    /**
     * Destructor.
     */
    ~ObjectTransformation();
    
    /**
     * Sets the base frame to convert the PbdObject messages to.
     * 
     * @param pBaseFrame The base coordinate frame to transform the objects to.
     */
    void setBaseFrame(std::string pBaseFrame);
    
    /**
     * Transforms an object into the given destination frame.
     */
    void transform(const boost::shared_ptr<pbd_msgs::PbdObject>& pObject);
    
  private:
    
    /**
     * The name of the frame the objects poses should be converted to.
     */
    std::string mBaseFrame;
    
    /**
     * A transformation listener that is responsible for transforming the evidences into the target coordinate system.
     */
    tf::TransformListener mTfListener;
    
  };
}