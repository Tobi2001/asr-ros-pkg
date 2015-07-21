#include "inference/model/SceneModelDescription.h"

namespace ProbabilisticSceneRecognition {

  SceneModelDescription::SceneModelDescription()
  {
  }
  
  SceneModelDescription::~SceneModelDescription()
  {
  }
  
  void SceneModelDescription::loadModelFromFile(std::string pPathToFile, std::string pAlgorithm)
  {
    // Check, if the file containing the model exist.
    if(!boost::filesystem::exists(pPathToFile))
      throw std::invalid_argument("Unable to procees loading. The model file doesn't exist!");
    
    // Status information for the user.
    ROS_INFO_STREAM("Loading scene model from this location: " << pPathToFile);
    
    // Load model file into memory.
    boost::property_tree::ptree pt;
    read_xml(pPathToFile, pt);
    
    // Recursively load all scenes found in the XML file.
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("psm.scenes"))
    {
      // Create a new fore- or background scene instance.
      boost::shared_ptr<SceneDescription> scene(new SceneDescription());
      
      // Let it load its parameters...
      scene->load(v.second, pAlgorithm);
      
      // ... and list it.
      mScenes.push_back(scene);
    }
  }
  
  void SceneModelDescription::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior)
  {
    // Iterate over all scenes and initialize their visualizers.
    for(boost::shared_ptr<SceneDescription> scene : mScenes)
	scene->initializeVisualizer(mSuperior);
  }
  
  void SceneModelDescription::integrateEvidence(const boost::shared_ptr<const pbd_msgs::PbdObject>& pObject)
  {
    // Add the evidence found to the buffer.
    mObjectEvidence.push(pObject);
  }
  
  void SceneModelDescription::integrateSceneGraph(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph)
  {
    // Forward the scene graph to the scenes.
    BOOST_FOREACH(boost::shared_ptr<SceneDescription> scene, mScenes)
      scene->update(pSceneGraph);
  }
  
  void SceneModelDescription::updateModel()
  {
    // Check if new evidences have been found since the last update.
    if(mObjectEvidence.hasWaitingEvidences())
    {
      // Integrate the evidences into what we already know.
      mObjectEvidence.update();
      
      // Get a list of ALL evidence...
      mObjectEvidence.getEvidences(mEvidenceList);
      
      // ... and forward it down the model.
      BOOST_FOREACH(boost::shared_ptr<SceneDescription> scene, mScenes)
	scene->update(mEvidenceList);
    }
  }
  
  void SceneModelDescription::getSceneListWithProbabilities(std::vector<SceneIdentifier>& pSceneList)
  {
    double sum = 0.0;
    
    pSceneList.clear();
    
    // Recalculate the probabilities of all scenes and collect their scene identifiers.
    BOOST_FOREACH(boost::shared_ptr<SceneDescription> scene, mScenes)
    {      
      // Recalculate probability.
      scene->calculateSceneProbaility();
      
      // Get the scene identifier and sum it up.
      SceneIdentifier i = *(scene->getSceneIdentifier());
      sum += i.mLikelihood;
      
      // Add the identifier to the list.
      pSceneList.push_back(i);
    }
    
    // Normalize the probabilities.
    if(sum > 0.0)
      for(unsigned int i = 0; i < pSceneList.size(); i++)
	pSceneList[i].mLikelihood /= sum;
  }
  
}