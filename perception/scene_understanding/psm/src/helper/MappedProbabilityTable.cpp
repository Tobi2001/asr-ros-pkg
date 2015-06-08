#include "helper/MappedProbabilityTable.h"

namespace ProbabilisticSceneRecognition {

  MappedProbabilityTable::MappedProbabilityTable()
  {
    // Initialize pointer.
    mTable.reset(new ProbabilityTable());
  }
  
  MappedProbabilityTable::MappedProbabilityTable(boost::property_tree::ptree& pPt)
  {
    // Initialize pointer.
    mTable.reset(new ProbabilityTable());
    
    // Execute the loading of mapping and probability table.
    load(pPt);
  }
  
  MappedProbabilityTable::~MappedProbabilityTable()
  {
    
  }
  
  void MappedProbabilityTable::load(boost::property_tree::ptree& pPt)
  {
    // Load mappings from object types to table indices.
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pPt.get_child("mapping"))
    {
      // Only access the 'map' child nodes.
      if(!std::strcmp(v.first.c_str(), "map"))
      {
	// Extract key and value and add them to the mapping.
 	unsigned int value = v.second.get<unsigned int>("<xmlattr>.id");
	std::string key = v.second.get<std::string>("<xmlattr>.name");

	// Check, if there is an invalid mapping to type zero (reserved for unknown types).
	if(value == 0)
	  throw std::invalid_argument("Unable to procees loading. Scene object model: no mapping to type 0 allowed, it is reserved for unknown objects");
	
	// Check, if there was a scene object type specified.
	if(key.size() == 0)
	  throw std::invalid_argument("Unable to procees loading. Scene object model: No name for object mapping specified.");
	
	// Add mapping to list.
	mMappingTypeToIndice.insert(std::pair<std::string, unsigned int>(key, value));
      }
    }
    
    // Load the probability table.
    mTable->load(pPt);
  }

  void MappedProbabilityTable::save(boost::property_tree::ptree& pPt)
  {
    // Create a subtree for the mapping.
    boost::property_tree::ptree subtreeMapping;
    
    // Iterate over the mapping of object types to indices.    
    for(std::map<std::string, unsigned int>::iterator it = mMappingTypeToIndice.begin(); it != mMappingTypeToIndice.end(); ++it)
    {
      boost::property_tree::ptree subtreeMap;
      
      subtreeMap.add("<xmlattr>.id", it->second); 
      subtreeMap.add("<xmlattr>.name", it->first);
      
      subtreeMapping.add_child("map", subtreeMap);
    }
    
    // Add subtree to main tree.
    pPt.add_child("mapping", subtreeMapping);
    
    // Save the probability table.
    mTable->save(pPt);
  }
  
   
  void MappedProbabilityTable::initializeTable(unsigned int pRows)
  {
    // Add one additional column to handle the unknown object class.
    mTable.reset(new ProbabilityTable(pRows, mMappingTypeToIndice.size() + 1));
  }
  
  double MappedProbabilityTable::getProbability(unsigned int pRow, std::string pType)
  {
    double result = 0;
    
    // If object type is known, return the associated probability. Otherwise return the default class probability.
    if(getIndex(pType) == 0)
      result = mTable->getProbability(pRow, 0);
    else
      result = mTable->getProbability(pRow, mMappingTypeToIndice[pType]);
    
    // Return the probability
    return result;
  }
  
  void MappedProbabilityTable::add(std::string pType)
  {    
    // If no entry was found, create a new one.
    if(getIndex(pType) == 0)
      mMappingTypeToIndice.insert(std::pair<std::string, unsigned int>(pType, mMappingTypeToIndice.size() + 1));
  }
  
  void MappedProbabilityTable::add(unsigned int pRow, std::string pType)
  {
    // If object not found (doubt that this will happen), add to unseen objects.
    // Otherwise add count tto the column associated with the object.
    if(getIndex(pType) == 0)
      mTable->add(pRow, 0, 1);
    else
      mTable->add(pRow, mMappingTypeToIndice[pType], 1);
  }
  
  void MappedProbabilityTable::setDefaultClassCounter(unsigned int pRow, double pCount)
  {
    mTable->add(pRow, 0, pCount);
  }
  
  void MappedProbabilityTable::normalize()
  {
    mTable->normalize();
  }
  
  unsigned int MappedProbabilityTable::getIndex(std::string pType)
  {
    unsigned int index = 0;
    
    // Get index, if in list.
    std::map<std::string, unsigned int>::iterator it = mMappingTypeToIndice.find(pType);
    if((it) != mMappingTypeToIndice.end())
    {
      index = it->second;
    }
    return index;
  }
  
  unsigned int MappedProbabilityTable::getNumberOfColumns()
  {
   return mTable->getNumberOfColumns();
  }
  
  unsigned int MappedProbabilityTable::getNumberOfRows()
  {
    return mTable->getNumberOfRows();
  }
  
}