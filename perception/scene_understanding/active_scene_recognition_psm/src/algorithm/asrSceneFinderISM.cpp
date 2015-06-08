#include "asrSceneFinderISM.h"
#include <ISM/MathHelper.hpp>

namespace ASR {

#define FILTERTHRESHOLD 0.0001

asrSceneFinderISM::asrSceneFinderISM(Eigen::Vector2i roimin,
                               Eigen::Vector2i roimax) :
    RoIMin(roimin),
    RoIMax(roimax),
    isCalculationFinished(true)
{
    ros::NodeHandle n("~");

    if(!n.getParam("dbfilename", dbfilename))
          dbfilename = "/home/staff/meissner/devel/ros/stacks/ilcasRosPkg/branches/renoreckling/recognition/scene_recognition/recordings/pascal/rcord.sqlite.smacksTest";

    if(!n.getParam("sensitivity", sensitivity))
          sensitivity = 0.05;

    recognizer = ISM::RecognizerPtr(new ISM::Recognizer(dbfilename, sensitivity));
    tableHelper = ISM::TableHelperPtr(new ISM::TableHelper(dbfilename));

    if(tableHelper->getModelPatternNames().empty())
    {
          ROS_ERROR("Loaded empty sql table into ism.");
          std::exit(1);
    }

    //Just extract those objects from table, that are non-reference objects.
    mObjectsInScenes = tableHelper->getObjectTypesAndIds();
    for(std::set<std::pair<std::string, std::string> >::iterator objectIterator = mObjectsInScenes.begin(); objectIterator != mObjectsInScenes.end(); objectIterator++)
    {
        if(objectIterator->first.find("_sub") != std::string::npos)
        {
            mObjectsInScenes.erase(objectIterator);
            objectIterator = mObjectsInScenes.begin();
        }
    }

    for(auto& object : mObjectsInScenes)
          ROS_INFO_STREAM("Objects in model: " << std::endl << object.first << " " << object.second << std::endl);

    //Extract all pattern names from database.
    rootPatternNames = tableHelper->getModelPatternNames();
    for(std::vector<std::string>::iterator patternIterator = rootPatternNames.begin(); patternIterator != rootPatternNames.end(); patternIterator++)
    {
        if(patternIterator->find("_sub") != std::string::npos)
        {
            rootPatternNames.erase(patternIterator);
            patternIterator = rootPatternNames.begin();
        }
    }

    objectSet = ISM::ObjectSetPtr(new ISM::ObjectSet());

    for(auto& patternName : tableHelper->getModelPatternNames())
    {
          std::set<std::pair<std::string, std::string> > objectsInPattern = tableHelper->getObjectTypesAndIdsBelongingToPattern(patternName);
          ISM::ObjectToVoteMap objectVotes = tableHelper->getVoteSpecifiersForPatternAndObjects(patternName, objectsInPattern);
          votes[patternName] = objectVotes;
    }

}


void asrSceneFinderISM::onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg)
{
    if(!isCalculationFinished)
    {
        // There is a calculation already running. Ignore object msg
        return;
    }
    if(!isObjectOfInterest(msg))
    {
        ROS_DEBUG("PbdObject msg received but recognition not of interest!");
        return;
    }


    // Object msg received -> do update scene hypothesis
    isCalculationFinished = false;

    //Transform pose of localized object from camera frame to ptu frame.
    poseInCamFrame.pose = msg->poseEstimation.pose;
    tf.transformPose(poseInPTUFrame.header.frame_id, poseInCamFrame, poseInPTUFrame);

    ISM::ObjectPtr newObject = convertPbdObjectToISMObject(msg, poseInPTUFrame.pose);

    // Check if the object has been recognized before
    if(mFoundObjects.find(std::make_pair(newObject->getType(), newObject->getId())) == mFoundObjects.end())
    {
        ROS_INFO("Previously unfound object with type = %s and id = %s is localized at (%.2f, %.2f, %.2f) resp. in left img: (%d, %d).\n", newObject->getType().c_str(), newObject->getId().c_str(), newObject->getPosePtr()->getPointPtr()->getX(), newObject->getPosePtr()->getPointPtr()->getY(), newObject->getPosePtr()->getPointPtr()->getZ(), msg->leftImageCentroid[0], msg->leftImageCentroid[1]);
        //Add previously not detected object to input for ism scene recognition.
        objectSet->insert(newObject);
    }


    // Calculate Scene Hypotheses
    const int numberOfScenes = 1;
    std::vector<ISM::RecognitionResultPtr> sceneList;
    sceneList = findScenes(FILTERTHRESHOLD, numberOfScenes);


    // Calculate Hyopthesis and unfound objects
    // The hypotheses are stored in this->hypos, unfound objects in mUnfoundObjects
    hypotheses.clear();
    mUnfoundObjects.clear();
    updateUnfoundObjectsPoseHypotheses(sceneList.at(0));

    isCalculationFinished = true;
}

bool asrSceneFinderISM::isObjectOfInterest(const pbd_msgs::PbdObject::ConstPtr& msg)
{
    //Transform pose of localized object from camera frame to ptu frame.
    poseInCamFrame.pose = msg->poseEstimation.pose;
    tf.transformPose(poseInPTUFrame.header.frame_id, poseInCamFrame, poseInPTUFrame);

    //Filter false positives from object recognition systems.
    const bool validRecognitionResult = msg->rightImageCentroid[0] >= RoIMin.x()
            && msg->leftImageCentroid[0] <= RoIMax.x()
            && msg->leftImageCentroid[1] >= RoIMin.y()
            && msg->leftImageCentroid[1] <= RoIMax.y();

    //Only process this detected object if it belongs to any of the scenes we are looking for.
    return validRecognitionResult && isObjectInScenes(msg->type, msg->identifier);
}


ISM::ObjectPtr asrSceneFinderISM::convertPbdObjectToISMObject(const pbd_msgs::PbdObject::ConstPtr& msg, geometry_msgs::Pose p)
{
    //Represent result from object recognition systems in ism recognition manner.
    return ISM::ObjectPtr( new ISM::Object(msg->type,
                                           new ISM::Pose(new ISM::Point(p.position.x, p.position.y, p.position.z),
                                                         new ISM::Quaternion((double)p.orientation.w, (double)p.orientation.x, (double)p.orientation.y, (double)p.orientation.z)),
                                           msg->identifier));
}

bool asrSceneFinderISM::lessConfidence(ISM::RecognitionResultPtr i,ISM::RecognitionResultPtr j)
{
    return (i->getConfidence() < j->getConfidence());
}


std::vector<ISM::RecognitionResultPtr> asrSceneFinderISM::findScenes(double filterThreshold, int resultsPerPattern)
{
    ISM::ObjectSetPtr tmpObjectSet(new ISM::ObjectSet(*objectSet));
    std::vector<ISM::RecognitionResultPtr> recognitionResults;

    //For every scene type (pattern name), return the root of the recognized scene model rated best of the given object set (concerning its confidence).
    recognitionResults = recognizer->recognizePattern(tmpObjectSet, filterThreshold, resultsPerPattern);

    //Sort in ascending order using recognition results confidence.
    std::sort(recognitionResults.begin(), recognitionResults.end(), lessConfidence);

    return recognitionResults;
}

void asrSceneFinderISM::updateUnfoundObjectsPoseHypotheses(ISM::RecognitionResultPtr referenceScene)
{
    ROS_ASSERT(referenceScene);
    ISM::PosePtr tmp = referenceScene->getReferencePosePtr();

    calcUnfoundPoses(tmp, referenceScene->getPatternName(), 1);
}


void asrSceneFinderISM::calcUnfoundPoses(ISM::PosePtr& referencePose, std::string patternName, unsigned int weight)
{
    ROS_ASSERT(referencePose);


    std::set<std::pair<std::string, std::string> > objectsInPattern = tableHelper->getObjectTypesAndIdsBelongingToPattern(patternName);

    for(auto& object : objectsInPattern)
    {
        //std::cout << "Taking votes for object " << object.first << std::endl;

        //Look whether we have an unfound object (reference object can never be found as they do not exist). Else ignore vote, as we already know its pose.
        if(mFoundObjects.find(std::make_pair(object.first, object.second)) == mFoundObjects.end())
        {

            //Get all votes that fit to combination of this reference (to reference and non-reference objects) and unfound object.
            std::vector<ISM::VoteSpecifierPtr> specifiers = votes.at(patternName).at(object.first).at(object.second);

            //If we have a reference object, search under it for unknown objects
            if(object.first.find("_sub") != std::string::npos)
            {
                //Go through all fitting votes.
                for(auto& specifier: specifiers)
                {
                    //Calculate absolute pose of unknown object
                    ISM::PointPtr absPosition = ISM::MathHelper::getOriginPoint(referencePose, specifier->refToObjectQuat, specifier->radius);
                    //Calculate full pose for next call of this function.
                    ISM::PosePtr absPose = ISM::MathHelper::getReferencePose(referencePose, absPosition, specifier->refToObjectPoseQuat);

                    //Check whether this reference object is identical to the reference of the ism in which it is object.
                    if(ISM::Recognizer::poseEqual(referencePose, absPose))
                    {
                        //Every single vote for a hypotheses in specifier has to be repreated for all redundancies in lower parts of the tree.
                        unsigned int numberOfHypotheses = weight * specifiers.size();

                        //One level higher in the scene model hierarchy, object pose hypotheses should be replicated for every vote in specifiers as well as every time this function would have been called. Reference object in this ism gets reference of next higher ism.
                        calcUnfoundPoses(absPose, object.first, numberOfHypotheses);

                        //If position of this object in scene is equal to pose of reference, then it has been chosen as reference and its pose will be equal to the reference through all votes. So we do not need to process any other votes here.
                        break;
                    }
                    //This reference object is not equal to the reference of the ism in which it resides.
                    else
                    {
                        //One level higher in the scene model hierarchy, object pose hypotheses should be replicated for every time this function would have been called.
                        calcUnfoundPoses(absPose, object.first, weight);
                    }
              }
            }
            //We have a non-reference object that we have not found and save its position.
            else
            {
                //Go through all fitting votes.
                for(auto& specifier: specifiers)
                {
                    //Calculate absolute pose of unknown object
                    ISM::PointPtr absPosition = ISM::MathHelper::getOriginPoint(referencePose, specifier->refToObjectQuat, specifier->radius);
                    ISM::PosePtr absPose = ISM::MathHelper::getReferencePose(referencePose, absPosition, specifier->refToObjectPoseQuat);

                    //Visualizer::pointToSphere(absPosition, 1,1,1,"anywoulddo",2);
                    //ROS_DEBUG_STREAM("absPositions: " << absPosition <<  std::endl);

                    hypotheses.push_back(AttributedPoint(specifier->objectType, ISMPoseToGeometryMsgPose(absPose)));

/*
                    //Check whether this non-reference object is identical to the reference of the ism in which it is object.
                    if(ISM::Recognizer::poseEqual(referencePose, absPose))
                    {
                        //Every single vote for a hypotheses in specifier has to be repreated for all redundancies in lower parts of the tree.
                        unsigned int numberOfHypotheses = weight * specifiers.size();

                        //As all votes are identical, just skip all further votes and insert points for all of them at once.
                        grid->insertPoint(specifier->objectType,
                                          specifier->observedId,
                                          Eigen::Vector3f(absPosition->getX(), absPosition->getY(), absPosition->getZ()),
                                          Eigen::Vector3f(referencePose->getPointPtr()->getX(), referencePose->getPointPtr()->getY(), referencePose->getPointPtr()->getZ()),
                                          numberOfHypotheses);

                        //Stop going through all further votes.
                        break;
                    }
                    //This non-reference object is not equal to the reference of the ism in which it resides.
                    else
                    {
                        //Insert absolute unkown object pose into tesselated sphere quad just for one vote, but considering all redundancies of the isms in lover parts of the tree
                        grid->insertPoint(specifier->objectType,
                                          specifier->observedId,
                                          Eigen::Vector3f(absPosition->getX(), absPosition->getY(), absPosition->getZ()),
                                          Eigen::Vector3f(referencePose->getPointPtr()->getX(), referencePose->getPointPtr()->getY(), referencePose->getPointPtr()->getZ()),
                                          weight);
                    }
*/
                }

                //Save that we found this object as missing in our currently considered scene hypotheses.
                mUnfoundObjects.insert(std::make_pair(object.first, object.second));
            }
        }
    }

}





}
