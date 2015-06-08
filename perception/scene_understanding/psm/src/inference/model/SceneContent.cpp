#include "inference/model/SceneContent.h"

namespace ProbabilisticSceneRecognition {
 
  SceneContent::SceneContent()
  {
  }
  
  SceneContent::~SceneContent()
  {
  }
  
  double SceneContent::getSceneProbability()
  {
    return (mAlgorithm) ? mAlgorithm->getProbability() : 0.0;
  }
  
  void SceneContent::setInferenceAlgorithm(boost::shared_ptr<InferenceAlgorithm> pAlgorithm)
  {
    mAlgorithm = pAlgorithm;
  }
  
  void SceneContent::loadInferenceAlgorithm(boost::property_tree::ptree& pPt)
  {
    // Check, if there was a description specified.
    if(!mAlgorithm)
      throw std::invalid_argument("Unable to procees loading. No inference algorithm specified.");

    mAlgorithm->load(pPt);
  }
  
  void SceneContent::doInference(std::vector<pbd_msgs::PbdObject> pEvidenceList, std::ofstream& pRuntimeLogger)
  {
    // Check, if there was a description specified.
    if(!mAlgorithm)
      throw std::invalid_argument("Unable to execute inference. No inference algorithm specified.");

    mAlgorithm->doInference(pEvidenceList, pRuntimeLogger);
  }
  
}