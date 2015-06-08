#pragma once

// Global includes
#include <string>

// Package includes
#include <boost/property_tree/ptree.hpp>

namespace ProbabilisticSceneRecognition {
  
  /**
   * This class wrapps a scene's identity
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class SceneIdentifier {
  public:
    
    /**
     * Constructor.
     */
    SceneIdentifier();
    
    /**
     * Constructor.
     * 
     * @param pType The type of the scene content wrapped by this class.
     * @param pDescription A short and meaningful description of the scene.
     */
    SceneIdentifier(std::string pType, std::string pDescription);
    
    /**
     * Copy constructor.
     * 
     * @param pOther The instance to copy.
     */
    SceneIdentifier(const SceneIdentifier& pOther);
    
    /**
     * Destructor.
     */
    ~SceneIdentifier();
    
    /**
     * Loads the scene identifier from XML.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Saves the scene identifier from XML.
     * 
     * @param pPt Data structure for performing XML operations.
     */
    void save(boost::property_tree::ptree& pPt);
    
  public:
    
    /**
     * The type of the scene content wrapped by this class.
     */
    std::string mType;
    
    /**
     * A short and meaningful description of the scene (e.g. breakfast, lunch, ...).
     */
    std::string mDescription;
    
    /**
     * The a priori plobability of the scene.
     */
    double mPriori;
    
    /**
     * The current probability of the scene.
     */
    double mLikelihood;
  };
}