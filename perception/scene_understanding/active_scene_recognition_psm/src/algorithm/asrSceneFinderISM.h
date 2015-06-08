#pragma once

#include <pbd_msgs/PbdObject.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>

#include <ISM/Recognizer.hpp>
#include <ISM/ObjectSet.hpp>
#include "../AttributedPoint.h"

namespace ASR
{


class asrSceneFinderISM
{
private:
    //image area boundary
    Eigen::Vector2i RoIMin;
    Eigen::Vector2i RoIMax;

    //Pose information of localized objects in different coordinate frames.
    geometry_msgs::PoseStamped poseInCamFrame;
    geometry_msgs::PoseStamped poseInPTUFrame;

    tf::TransformListener tf;

    // The path to the db which contains the scenes to be recognized
    std::string dbfilename;

    // ISM Parameters
    ISM::RecognizerPtr recognizer;
    ISM::TableHelperPtr tableHelper;
    double sensitivity;
    // all recognized objects
    ISM::ObjectSetPtr objectSet;

    //Object types detected so far.
    std::set<std::pair<std::string, std::string> > mFoundObjects;
    //Object types detected so far.
    std::set<std::pair<std::string, std::string> > mUnfoundObjects;
    //Objects (type and id) that reside in table for scenes to be found.
    std::set<std::pair<std::string, std::string> > mObjectsInScenes;
    //Pattern names for all scenes in the model we loaded.
    std::vector<std::string> rootPatternNames;

    bool isCalculationFinished;

    ISM::PatternToObjectToVoteMap votes;


    /**
      Contains all hypotheses which were calculated.
      A hypothesis contains the object type and the expected pose.
      */
    std::vector<AttributedPoint> hypotheses;


    //ISM Helper
    static bool lessConfidence(ISM::RecognitionResultPtr i,ISM::RecognitionResultPtr j);

    /**
      Helper function for calculating the hypotheses
      */
    void calcUnfoundPoses(ISM::PosePtr& referencePose, std::string patternName, unsigned int weight);

    // ISM Functions
    /**
      Calculates the scnene probabilty.
      Returns them in ascending order.
    */
    std::vector<ISM::RecognitionResultPtr> findScenes(double filterThreshold,
                                                    int resultsPerPattern);

    /**
      Calculates the hypotheses recursivly.
      These are stored in this->hypotheses.
      */
    void updateUnfoundObjectsPoseHypotheses(ISM::RecognitionResultPtr referenceScene);

    geometry_msgs::Pose ISMPoseToGeometryMsgPose(ISM::PosePtr p)
    {
        geometry_msgs::Pose pose;
        ISM::PointPtr position = p->getPointPtr();
        pose.position.x = position->getX();
        pose.position.y = position->getY();
        pose.position.z = position->getZ();

        return pose;
    }


public:
    asrSceneFinderISM(Eigen::Vector2i roimin = Eigen::Vector2i(323,969),
                   Eigen::Vector2i roimax = Eigen::Vector2i(241,723));

    /**
      Is called when an object is recognized.
      Checks if the recognition is valid and calculates object hypothesis.
      */
    void onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg);

    /**
      Checks if a given object is in the RoI and the scenes
      */
    bool isObjectOfInterest(const pbd_msgs::PbdObject::ConstPtr& msg);




    bool calculationFinished() {return isCalculationFinished;}


    /**
      Converts a PbdObject msg to a ISM ObjectPtr
      */
    ISM::ObjectPtr convertPbdObjectToISMObject(const pbd_msgs::PbdObject::ConstPtr& msg, geometry_msgs::Pose p);

    /**
      Checks if a given object is in the scenes
      */
    bool isObjectInScenes(std::string objectType, std::string objectIdentifier)
    {
        return mObjectsInScenes.find(std::make_pair(objectType, objectIdentifier)) != mObjectsInScenes.end();
    }

    /**
      Returns all pattern names
      */
    std::list<std::string> getRootPatterns()
    {
        return std::list<std::string>(rootPatternNames.begin(),rootPatternNames.end());
    }

    std::vector<AttributedPoint> getHypotheses()
    {
        return hypotheses;
    }

};

typedef boost::shared_ptr<asrSceneFinderISM> asrSceneFinderISMPtr;
}
