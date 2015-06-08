#include "asrSceneFinderPSM.h"
#include <inference/model/SceneModelDescription.h>

namespace ASR
{

using namespace ProbabilisticSceneRecognition;

asrSceneFinderPSM::asrSceneFinderPSM()
: mNodeHandle("~")
{
    SceneModelFileName = "/home/SMBAD/gehrung/home/Data/P1V2/psm_models/breakfast.xml";
    std::string inferenceAlgorithm = "maximum";

    mModel = boost::shared_ptr<ProbabilisticSceneRecognition::SceneModelDescription>
            (new ProbabilisticSceneRecognition::SceneModelDescription());

    // Initialize the transformations of objects into the given frame.
    mObjectTransform.setBaseFrame("/PTU");

    // Message for the user.
    ROS_INFO("Initializing psm inference engine.");

    // Load model with the path and select the inferece algorithm (maximum)
    mModel->loadModelFromFile(SceneModelFileName, inferenceAlgorithm);



    // The ROS topic for scene graph messages to listen to.
    std::string pPbdSceneGraphTopic = "/scene_graphs";
    // A list of paths to rosbag files containing PbdSceneGraph messages.
    XmlRpc::XmlRpcValue mInputBagFilenames;

    // Try to get names of bag files with PbdSceneGraph message input, if any exist.
        if(mNodeHandle.getParam("bag_filenames_list", mInputBagFilenames))
        {
          // Either one string or a list of strings is accepted as input.
          if(mInputBagFilenames.getType() == XmlRpc::XmlRpcValue::TypeString)
          {
        // When not called directly from CLI, we can used a list as input.
          } else {
        // Check whether parameter loaded is really a list of values.
        if(mInputBagFilenames.getType() != XmlRpc::XmlRpcValue::TypeArray)
          throw std::invalid_argument("CLI option \"bag_filenames_list\" not set with an array.");

        // Go through all potential filenames of PbdSceneGraph rosbag files.
        for(int i = 0; i < mInputBagFilenames.size(); i++)
        {
          // Check whether parameter list element is really a filename.
          if(mInputBagFilenames[i].getType() != XmlRpc::XmlRpcValue::TypeString)
            throw std::invalid_argument("Bag file path no. " + boost::lexical_cast<std::string>(i) + "is no valid string.");
        }
          }
        }

    // Register callback handlers for scene graphs which contain the raw data used for learning.
    mSceneGraphListener = mNodeHandle.subscribe(pPbdSceneGraphTopic, 5, &asrSceneFinderPSM::newSceneGraphCallback, this);

    // Read the learning data from bag file.
    readLearnerInputBags(mInputBagFilenames);
}

void asrSceneFinderPSM::onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg)
{
    // Buffers the evidence to keep callback time as short as possible.
    mEvidenceBuffer.push(convertPdbObjectMsg(msg));

    updateHypotheses();
}

void asrSceneFinderPSM::updateHypotheses()
{
    // Status information for the user.
    ROS_DEBUG("Updating psm inference engine.");

        /********************************************************************
         * Integrate the collected evidence into the model.
         ********************************************************************/

        // Process evidences!
        while(!mEvidenceBuffer.empty())
        {
          // Get the first entry.
          boost::shared_ptr<pbd_msgs::PbdObject> evidence = mEvidenceBuffer.front();

          // Remove the entry processed from the queue.
          mEvidenceBuffer.pop();

          // Status information for the user.
          ROS_INFO("Object of type '%s' found.", evidence->type.c_str());

          try
          {
              // Try to transform evidence into target coordinate system.
              mObjectTransform.transform(evidence);
          } catch(std::exception& exception)
          {
              // No transformation found, dropping object!
              ROS_INFO("Unable to resolve transformation in target coordinate frame. Dropping object.");
              continue;
          }

          // Forward the new evidence to the model.
          mModel->integrateEvidence(evidence);
        }

    // Update the model with the evidence collected until now.
    mModel->updateModel();

    /********************************************************************
     * Integrate the learning data loaded from bag file.
     ********************************************************************/

    // Process scene graphs!
    while(!mSceneGraphBuffer.empty())
    {
        // Get the first entry.
        boost::shared_ptr<const pbd_msgs::PbdSceneGraph> sceneGraph = mSceneGraphBuffer.front();

        // Remove the entry processed from the queue.
        mSceneGraphBuffer.pop();

        // Status information for the user.
        ROS_INFO_STREAM("SceneGraph of type '" << sceneGraph->identifier << "' found.");

        // Forward evidence to the model.
        mModel->integrateSceneGraph(sceneGraph);
    }


    /********************************************************************
     * Do the inference and show the results.
     ********************************************************************/

    // Get the results and show them.
    std::vector<ProbabilisticSceneRecognition::SceneIdentifier> pSceneList;
    mModel->getSceneListWithProbabilities(pSceneList);

    printf("===========================================");
    printf("This are the scene probabilities:\n");
    for(ProbabilisticSceneRecognition::SceneIdentifier i : pSceneList)
        printf(" -> %s (%s): %f (%f)\n", i.mDescription.c_str(), i.mType.c_str(), i.mLikelihood, i.mPriori);
}


const boost::shared_ptr<pbd_msgs::PbdObject> asrSceneFinderPSM::convertPdbObjectMsg(const pbd_msgs::PbdObject::ConstPtr& msg)
{
    const boost::shared_ptr<pbd_msgs::PbdObject> pObject = boost::shared_ptr<pbd_msgs::PbdObject>(new pbd_msgs::PbdObject());

    pObject->type = msg->type;
    pObject->boundingBox = msg->boundingBox;
    pObject->color = msg->color;
    pObject->colorName = msg->colorName;
    pObject->header = msg->header;
    pObject->identifier = msg->identifier;
    pObject->leftImageCentroid = msg->leftImageCentroid;
    pObject->leftImageName = msg->leftImageName;
    pObject->meshResourcePath = msg->meshResourcePath;
    pObject->poseEstimation = msg->poseEstimation;
    pObject->providedBy = msg->providedBy;
    pObject->rightImageCentroid = msg->rightImageCentroid;
    pObject->rightImageName = msg->rightImageName;
    pObject->sizeConfidence = msg->sizeConfidence;
    pObject->typeConfidence = msg->typeConfidence;

    return pObject;
}

void asrSceneFinderPSM::newSceneGraphCallback(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph)
{
    // Buffers the scene graph to keep callback time as short as possible.
    mSceneGraphBuffer.push(pSceneGraph);
}



void asrSceneFinderPSM::extractPbdSceneGraphsFromBag(const std::string& pPbdSceneGraphsBagPath)
  {
    ROS_INFO_STREAM("Extracting PbdSceneGraph messages from rosbag file: " << pPbdSceneGraphsBagPath);

    // Check whether topic name for scene graph has been set before trying to parse rosbag files.
    if(!mSceneGraphListener)
      throw std::logic_error("Cannot parse bag file with PbdSceneGraphs without knowing on which topic they were sent.");

    // Set topic representatio. When parsing rosbag files this is required for extracting the messages which are representing scene graphs.
    rosbag::TopicQuery pbdSceneGraphIdentifier(mSceneGraphListener.getTopic());

    // Create file handler for rosbag file to be read.
    rosbag::Bag pbdSceneGraphsBag;

    // Get read-only access to messages in given rosbag file, create access infrastructure.
    try {
      pbdSceneGraphsBag.open(pPbdSceneGraphsBagPath, rosbag::bagmode::Read);
    } catch(rosbag::BagIOException& exception) {
      // ROS_ERROR does not work here.
      std::cerr << "Trying to extract PbdSceneGraph messages aborted because of: " << exception.what() << std::endl;
      // Quit this function as no data is to be processed.
      return;
    }

    // Create interface to extract only scene graph messages in given bag file from a previously defined topic.
    rosbag::View pbdSceneGraphView(pbdSceneGraphsBag, pbdSceneGraphIdentifier);

    // Check whether there is any raw data from a scene on the topic where we expect them.
    if(!pbdSceneGraphView.size())
      ROS_WARN_STREAM("No PbdSceneGraph messages exist in " << pPbdSceneGraphsBagPath << " on topic " << mSceneGraphListener.getTopic() << ".");

    // Get access to all scene graphs in bag file to transfer them to parameter learner for scene model.
    for(rosbag::View::iterator sceneGraphIterator = pbdSceneGraphView.begin(); sceneGraphIterator != pbdSceneGraphView.end(); sceneGraphIterator++) {

      // Get interface compliant to PbdSceneGraph message on rosbag item currently taken into account.
      pbd_msgs::PbdSceneGraph::ConstPtr currentSceneGraph = sceneGraphIterator->instantiate<pbd_msgs::PbdSceneGraph>();

      // Success check for extraction.
      if(currentSceneGraph != NULL)
      // And add all object measurements in scene graph to parameter learners.
    newSceneGraphCallback(currentSceneGraph);
    }

    // Clean up.
    pbdSceneGraphsBag.close();
  }

void asrSceneFinderPSM::readLearnerInputBags(XmlRpc::XmlRpcValue pInputBagFilenames)
  {
    // If only one string is given to node, just use this as path to scene graphs.
    // Otherwise load a bunch of files and process input as it was one file.
    if(pInputBagFilenames.getType() == XmlRpc::XmlRpcValue::TypeString) {
      extractPbdSceneGraphsFromBag(static_cast<std::string>(pInputBagFilenames));
    } else {

      // Go through all paths to PbdSceneGraph rosbag files passed to ros node via cli.
      // Extract all PbdSceneGraph messages from rosbag file currently taken into account.
      for(int i = 0; i < pInputBagFilenames.size(); i++)
    extractPbdSceneGraphsFromBag(static_cast<std::string>(pInputBagFilenames[i]));
    }
  }


}
