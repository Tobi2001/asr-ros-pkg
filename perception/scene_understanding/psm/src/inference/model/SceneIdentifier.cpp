#include "inference/model/SceneIdentifier.h"

namespace ProbabilisticSceneRecognition {
 
  SceneIdentifier::SceneIdentifier()
  {
  }
  
  SceneIdentifier::SceneIdentifier(std::string pType, std::string pDescription)
  : mType(pType)
  , mDescription(pDescription)
  {
  }
  
  SceneIdentifier::SceneIdentifier(const SceneIdentifier& pOther)
  {
    mType = pOther.mType;
    mDescription = pOther.mDescription;
    mPriori = pOther.mPriori;
    mLikelihood = pOther.mLikelihood;
  }
  
  SceneIdentifier::~SceneIdentifier()
  {
  }
  
  void SceneIdentifier::load(boost::property_tree::ptree& pPt)
  {
    mDescription = pPt.get<std::string>("<xmlattr>.name");
    mType = pPt.get<std::string>("<xmlattr>.type");
    mPriori = pPt.get<double>("<xmlattr>.priori");
  }
  
  void SceneIdentifier::save(boost::property_tree::ptree& pPt)
  {
    pPt.add("<xmlattr>.name", mDescription);
    pPt.add("<xmlattr>.type", mType);
    pPt.add("<xmlattr>.priori", mPriori);
  }
    
}