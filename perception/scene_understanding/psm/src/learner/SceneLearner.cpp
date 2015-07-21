#include "learner/SceneLearner.h"

namespace ProbabilisticSceneRecognition {

  SceneLearner::SceneLearner(std::string pSceneName)
  : mSceneName(pSceneName)
  {
    
  }
  
  SceneLearner::~SceneLearner()
  {
  }
  
  bool SceneLearner::isExampleForScene(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pExample)
  {
    return mSceneName.compare(pExample->identifier) == 0;
  }
  
  void SceneLearner::addExampleToScene(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pExample)
  {
    mExamplesList.push_back(pExample);
  }
  
  void SceneLearner::setVolumeOfWorkspace(double pValue)
  { 
    mWorkspaceVolume = pValue;
  }
  
  void SceneLearner::setPriori(double pPriori)
  {
    mPriori = pPriori;
  }
    
}