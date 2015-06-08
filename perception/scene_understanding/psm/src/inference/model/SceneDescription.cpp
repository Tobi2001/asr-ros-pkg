#include "inference/model/SceneDescription.h"

namespace ProbabilisticSceneRecognition {
 
  SceneDescription::SceneDescription()
  {
    mIdentifier.reset(new SceneIdentifier());
  }
  
  SceneDescription::~SceneDescription()
  {
    // Close the file used for logging the runtime of this scene.
    if(mRuntimeFile.is_open())
      mRuntimeFile.close();
  }
  
  void SceneDescription::load(boost::property_tree::ptree& pPt, std::string pAlgorithm)
  {
    // Load scene identifier.
    mIdentifier->load(pPt);
    
    // Check, if there was a description specified.
    if(mIdentifier->mDescription.size() == 0)
      throw std::invalid_argument("Unable to procees loading. No description for scene specified.");
    
    // Check, if there was a scene object type specified.
    if(mIdentifier->mType.size() == 0)
      throw std::invalid_argument("Unable to procees loading. No description for scene '" + mIdentifier->mDescription + "' spedified.");
    
    // Check, if there the a priori probability is valid.
    if(mIdentifier->mPriori < 0.0 || mIdentifier->mPriori > 1.0)
      throw std::invalid_argument("Unable to procees loading. The a priori probability of scene '" + mIdentifier->mDescription + "' is invalid (value: " + boost::lexical_cast<std::string>(mIdentifier->mPriori) + ").");
    
    // Create a new scene content of the given type.
    if(!std::strcmp(mIdentifier->mType.c_str(), "ocm")) {
      ROS_INFO_STREAM("Loading foreground scene with type '" << mIdentifier->mType << "' and name '" << mIdentifier->mDescription << "'.");
      
      // Create a new ocm foreground scene content.
      mContent.reset(new ForegroundSceneContent());
    } else if(!std::strcmp(mIdentifier->mType.c_str(), "background")) {
      ROS_INFO_STREAM("Loading background scene of type '" << mIdentifier->mType << "'.");
      
      // Create a new background scene content.
      mContent.reset(new BackgroundSceneContent());
    } else {
      throw std::invalid_argument("Unable to procees loading. Can't create scene content of the unknown type '" + mIdentifier->mType + "'.");
    }
    
    // Load scene content and initialize the inference algorithm.
    mContent->initializeInferenceAlgorithms(pAlgorithm);
    mContent->load(pPt);
    
    // Initialize output file for runtime evaluation results.
    std::string runtimeLogPath;
    ros::NodeHandle mNodeHandle("~");
    if(mNodeHandle.getParam("runtime_log_path", runtimeLogPath))
    {
      mRuntimeFile.open(runtimeLogPath + "/" + mIdentifier->mDescription + ".csv");
      mRuntimeFile << "sceneobject,runtime\n";
    }
  }
  
  void SceneDescription::initializeVisualizer(boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mSuperior)
  {
    // Debug message.
    ROS_INFO_STREAM("Initializing visualizer for scene '" << mIdentifier->mDescription << "'.");
    
    // Create a new coordinator for primary scene object visualization.
    mVisualizer.reset(new Visualization::ProbabilisticSceneVisualization(mIdentifier->mDescription));
    
    // Append it to supperior visualizer.
    mSuperior->appendVisualizer(mVisualizer);
    
    // Forward the visualizer to the scene content.
    if(mContent)
      mContent->initializeVisualizer(mVisualizer);
  }
  
  void SceneDescription::update(std::vector<pbd_msgs::PbdObject> pEvidenceList)
  {
    // Debug message.
    ROS_INFO_STREAM("Evaluating scene '" << mIdentifier->mDescription << "'.");
    
    // Trigger the update process.
    if(mContent)
      mContent->update(pEvidenceList, mRuntimeFile);
  }
  
  void SceneDescription::update(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph)
  {
    if(mContent)
      mContent->update(pSceneGraph);
  }
  
  void SceneDescription::calculateSceneProbaility()
  {
    double result = 0.0;
    
    if(mContent)
      result = mContent->getSceneProbability();
    
    // Debug message.
    ROS_INFO_STREAM("The relative probability for scene '" << mIdentifier->mDescription << "' is '" << result << "'.");
    
    // Return P(D|S) * P(S)
    mIdentifier->mLikelihood = result * mIdentifier->mPriori;
  }
  
  boost::shared_ptr<SceneIdentifier> SceneDescription::getSceneIdentifier()
  {
    return boost::shared_ptr<SceneIdentifier>(new SceneIdentifier(*mIdentifier));
  }
  
  void SceneDescription::setSceneIdentifier(boost::shared_ptr<SceneIdentifier> pIdentifier)
  {
    mIdentifier = pIdentifier;
  }
  
}