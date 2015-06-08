#pragma once

// Global includes
#include <map>
#include <string>
#include <vector>

// Package includes
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

// Local includes
#include "helper/SerializationHelper.h"

namespace ProbabilisticSceneRecognition {
  
  /**
   * A probability table that handles learning, queries and persistence.
   *
   * @author Joachim Gehrung
   * @version See SVN
   */
  class ProbabilityTable {
  public:

    /**
     * Constructor.
     */
    ProbabilityTable();
    
    /**
     * Constructor.
     * 
     * @param pRows The number of rows.
     * @param pColums The number of columns.
     */
    ProbabilityTable(unsigned int pRows, unsigned int pColums);
    
    /**
     * Constructor.
     * 
     * @param pPt The data structure used to load the container from XML.
     */
    ProbabilityTable(boost::property_tree::ptree& pPt);
    
    /**
     * Destructor.
     */
    ~ProbabilityTable();
    
    /**
     * Loads the content from XML.
     * 
     * @param pPt Data structure for handling XML operations.
     */
    void load(boost::property_tree::ptree& pPt);
    
    /**
     * Saves the content to XML.
     * 
     * @param pPt Data structure for handling XML operations.
     */
    void save(boost::property_tree::ptree& pPt); 
    
    /**
     * Returns the probability for the given object type.
     * 
     * @param pRow The number of the row.
     * @param pColumn The number of the column.
     * @return Returns the probability for the given entry.
     */
    double getProbability(unsigned int pRow, unsigned int pColumn);
    
    /**
     * Returns the number of columns in the probability table.
     */
    unsigned int getNumberOfColumns();
    
    /**
     * Returns the number of rows in the probability table.
     */
    unsigned int getNumberOfRows();
    
    /**
     * Adds the given number of counts to the given entry.
     * 
     * @param pRow The number of the row.
     * @param pColumn The number of the column.
     * @param pCounts The number of counts to add.
     */
    void add(unsigned int pRow, unsigned int pColumn, unsigned int pCounts);
    
    /**
     * Normalizes the given probability table so that the values sum up to one.
     */
    void normalize();
    
  private:
    
    /**
     * The entries of the table. To be a valid probability distribution, they must sum up to one.
     */
    std::vector<std::vector<double> > mEntries;
  };
}