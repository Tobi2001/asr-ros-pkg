#include "inference/model/ObjectEvidence.h"

namespace ProbabilisticSceneRecognition {
 
  ObjectEvidence::ObjectEvidence()
  {
    ros::NodeHandle mNodeHandle("~");
    
    // Try to get the timeout for object evidences.
    if(!mNodeHandle.getParam("evidence_timeout", mTimeout))
      throw std::runtime_error("Please specify parameter " + std::string("evidence_timeout") + " when starting this node.");
  }
  
  ObjectEvidence::~ObjectEvidence()
  {
  }

  void ObjectEvidence::push(const boost::shared_ptr<const pbd_msgs::PbdObject>& pObject)
  {
    mBuffer.push_back(*pObject);
  }

  bool ObjectEvidence::hasWaitingEvidences()
  {
    return mBuffer.size() > 0;
  }
  
  void ObjectEvidence::update()
  {
    // Iterate over all accumulated evidences and add each one to the list.
    // Also keep book about if a new object has been found.
    BOOST_FOREACH(pbd_msgs::PbdObject object, mBuffer)
    {
      // Create iterator for the outer index (object type).
      std::map<std::string, std::map<std::string, KalmanFilter> >::iterator it;
      
      // Exists an entry with the given object type?
      if((it = mObjectEvidences.find(object.type)) != mObjectEvidences.end())
      {
	// Create iterator for the inner index (object instance).
	std::map<std::string, KalmanFilter>::iterator it2;
	
	// Exists an entry with the given object instance name?
	if((it2 = it->second.find(object.identifier)) != it->second.end())
	{
	  // There exists an entry for type and instance, so we update the associated kalman filter.
	  it2->second.update(object);
	  
	  // Status information for the user.
	  ROS_DEBUG_STREAM("Object Evidence: replaced object (" << object.type << ", " << object.identifier << ").");
	} else {
	  
	  // Create a new kalman filter.
	  it->second.insert(std::pair<std::string, KalmanFilter>(object.identifier, KalmanFilter(object)));
	  
	  // Status information for the user.
	  ROS_DEBUG_STREAM("Object Evidence: object with new identifier found (" << object.type << ", " << object.identifier << ").");
	}
      } else {
	// There was no entry for the given object type and instance.
	// So we first add a map for the instance to the evidences and then an entry to this map.
	std::map<std::string, KalmanFilter> entry;
	entry.insert(std::pair<std::string, KalmanFilter>(object.identifier, KalmanFilter(object)));
	mObjectEvidences.insert(std::pair<std::string, std::map<std::string, KalmanFilter> >(object.type, entry));
	// THIS IS WHY MANY PEOPLE PREFER JAVA...! OR PYTHON ;D!
	
	// Status information for the user.
	ROS_DEBUG_STREAM("Object Evidence: object with new type and identifier found (" << object.type << ", " << object.identifier << ").");
      }
    }
    
    // Search for timed out evidences.
    std::map<std::string, std::map<std::string, KalmanFilter> >::iterator it;
    for(it = mObjectEvidences.begin(); it != mObjectEvidences.end(); ++it)
    {
      
      // Create iterator for the inner index (object instance).
      std::map<std::string, KalmanFilter>::iterator it2 = it->second.begin();
      if(it2 != it->second.end())
      {
	// Remove timed out evidence.
	if (it2->second.isTimedOut(mTimeout))
	{
	  pbd_msgs::PbdObject object = it2->second.getObject();
	  
	  it->second.erase(it2++);
	  ROS_DEBUG_STREAM("Removed timed out evidence (" << object.type << ", " << object.identifier << ").");
	} else {
	  ++it2;
	}
      }
    }
    
    // Delete all entries in the buffer.
    mBuffer.clear();
  }
  
  void ObjectEvidence::getEvidences(std::vector<pbd_msgs::PbdObject>& pEvidences)
  {    
    // Erase old evidences from history *dramatic drum roll*.
    pEvidences.clear();
    
    // Create iterators for object type and instance indices.
    std::map<std::string, std::map<std::string, KalmanFilter> >::iterator it;
    std::map<std::string, KalmanFilter>::iterator it2;
    
    // Iterate over the object type index (the outer index).
    for(it = mObjectEvidences.begin(); it != mObjectEvidences.end(); it++)
    {
      // Iterate over the object instance names (the inner index).
      for(it2 = it->second.begin(); it2 != it->second.end(); it2++)
      {
	// Add every evidence to the vector.
	pEvidences.push_back(it2->second.getObject());
      }
    }
  }
  
}
