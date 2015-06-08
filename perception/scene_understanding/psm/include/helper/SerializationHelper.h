#pragma once

// Global includes
#include <cmath>
#include <vector>

// Package includes
#include <Eigen/Core>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * Helper class for converting primitive datatypes into string representations and vice versa.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SerializationHelper {
  public:
    
    /**
     * Constructor.
     */
    SerializationHelper();
    
    /**
     * Destructor.
     */
    ~SerializationHelper();
    
    /**
     * Converts a vector of doubles into a string.
     * 
     * @param vec The vector to convert to string.
     * @param str The string to fill with the content of the vector.
     */
    static void convertVectorToString(std::vector<double>& vec, std::string& str);
    
    /**
     * Converts a vector of doubles into a string.
     * 
     * @param str The string to convert to a vector.
     * @param vec The vector to fill with the content of the string.
     */
    static void convertStringToVector(std::string& str, std::vector<double>& vec);
    
    /**
     * Converts an Eigen vector into a string.
     * 
     * @param vec The vector to convert to string.
     * @param str The string to fill with the content of the vector.
     */
    static void convertVectorToString(boost::shared_ptr<Eigen::VectorXd>& vec, std::string& str);
    
    /**
     * Converts a string into an Eigen vector.
     * 
     * @param size The size of the vector.
     * @param str The string to convert to a vector.
     * @param vec The vector to fill with the content of the string.
     */
    static void convertStringToVector(unsigned int size, std::string& str, boost::shared_ptr<Eigen::VectorXd>& vec);
    
    /**
     * Converts am Eigen matrix into a string.
     * 
     * @param mat The Eigen matrix to convert to string.
     * @param str The string to fill with the content of the matrix.
     */
    static void convertMatrixToString(boost::shared_ptr<Eigen::MatrixXd>& mat, std::string& str);
    
    /**
     * Converts a string into an Eigen matrix.
     * 
     * @param size The size of rows and colums of the matrix.
     * @param str The string to convert to an Eigen matrix.
     * @param mat The matrix to fill with the content of the string.
     */
    static void convertStringToMatrix(unsigned int size, std::string& str, boost::shared_ptr<Eigen::MatrixXd>& mat);
  };
}