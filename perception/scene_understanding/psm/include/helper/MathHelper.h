#pragma once

// Global includes
#include <cmath>
#include <vector>

// Package includes
#include <pl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <pbd_msgs/PbdNode.h>
#include <pbd_msgs/PbdObject.h>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Helper class for mathematical calculations.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class MathHelper {
  public:
    /**
     * Constructor.
     */
    MathHelper();
    
    /**
     * Destructor.
     */
    ~MathHelper();

    /**
     * Copies the content of a vector from ProBT to Eigen.
     * 
     * @param pFrom The ProBt vector to copy from.
     * @param pTo The Eigen vector to copy to.
     */
    static void copy(plFloatVector& pFrom, boost::shared_ptr<Eigen::VectorXd>& pTo);

    /**
     * Copies the content of a matrix from ProBT to Eigen.
     * 
     * @param pFrom The ProBt matrix to copy from.
     * @param pTo The Eigen matrix to copy to.
     */
    static void copy(plFloatMatrix& pFrom, boost::shared_ptr<Eigen::MatrixXd>& pTo);
    
  };
}