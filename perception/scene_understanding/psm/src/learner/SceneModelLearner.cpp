#include "learner/SceneModelLearner.h"

namespace ProbabilisticSceneRecognition {

  SceneModelLearner::SceneModelLearner(std::string pForegroundSceneModel, double pWorkspaceVolume, double pStaticBreakRatio, double pTogetherRatio, double pMaxAngleDeviation)
  : mForegroundSceneModel(pForegroundSceneModel)
  , mWorkspaceVolume(pWorkspaceVolume)
  , mStaticBreakRatio(pStaticBreakRatio)
  , mTogetherRatio(pTogetherRatio)
  , mMaxAngleDeviation(pMaxAngleDeviation)
  {
  }
  
  SceneModelLearner::~SceneModelLearner()
  {
  }
   
  void SceneModelLearner::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior)
  {
    for(boost::shared_ptr<ForegroundSceneLearner> learner : mSceneLearners)
	learner->initializeVisualizer(mSuperior);
  }
   
  void SceneModelLearner::addExample(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pExample)
  {
    // Add every example to background model.
    mBackgroundSceneLearner.addExampleToScene(pExample);
    
    // Search for the learner that handles the example based on it's scene.
    bool hasBeenAssigned = false;
    BOOST_FOREACH(boost::shared_ptr<SceneLearner> learner, mSceneLearners)
    {
      hasBeenAssigned |= learner->isExampleForScene(pExample);
      
      if(hasBeenAssigned)
      {
	learner->addExampleToScene(pExample);
	break;
      }
    }
    
    // If no learner was found, create a new one.
    if(!hasBeenAssigned)
    {
      if(mForegroundSceneModel.compare(OCM_SCENE_MODEL) == 0)
      {
	mSceneLearners.push_back(boost::shared_ptr<ForegroundSceneLearner>(new OcmForegroundSceneLearner(pExample)));
	
	ROS_INFO_STREAM("Scene model learner: Created a new scene model of type '" << OCM_SCENE_MODEL << "'.");
      } else {
	throw std::invalid_argument("Scene model learner can not instanciate a model of the unknown type '" + mForegroundSceneModel + "'.");
      }
    }
  }
  
  void SceneModelLearner::generateSceneModel()
  {
    ROS_INFO("Scene model learner: Generating scene model.");
    
    // Background scene should learn it's parameters.
    mBackgroundSceneLearner.setVolumeOfWorkspace(mWorkspaceVolume);
    mBackgroundSceneLearner.learn();
    
    // Each foreground scene shall learn it's parameters.
    BOOST_FOREACH(boost::shared_ptr<ForegroundSceneLearner> learner, mSceneLearners)
    {
      learner->setVolumeOfWorkspace(mWorkspaceVolume);
      learner->setClusteringParameters(mStaticBreakRatio, mTogetherRatio, mMaxAngleDeviation);
      learner->learn();
    }
    
    // Set an equal distributed a priori probability for every scene.
    // The number of scenes is the number of all foreground scene plus the background scene.
    double priori = 1.0 / (mSceneLearners.size() + 1);
    BOOST_FOREACH(boost::shared_ptr<SceneLearner> learner, mSceneLearners)
      learner->setPriori(priori);
    mBackgroundSceneLearner.setPriori(priori);
  }
  
  void SceneModelLearner::saveSceneModelToFile(std::string pPathToFile)
  {
    ROS_INFO_STREAM("Scene model learner: Saving scene model to file: " << pPathToFile << ".");
    
    // This property tree represents the content of the XMl file.
    boost::property_tree::ptree pt;
    
    // Export the background scene.
    mBackgroundSceneLearner.save(pt);
    
    // Export all foreground scenes.
    BOOST_FOREACH(boost::shared_ptr<SceneLearner> learner, mSceneLearners)
      learner->save(pt);
    
    // Write the property tree to file.
    write_xml(pPathToFile, pt);
  }
  
}