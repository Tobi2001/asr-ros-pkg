#pragma once

// Package includes
#include <ros/ros.h>
#include <pbd_msgs/PbdObject.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <queue>

#include "../AttributedPoint.h"

// Package includes
#include <ros/ros.h>
#include <rosbag/view.h>
#include <inference/model/SceneModelDescription.h>
#include <helper/ObjectTransformation.h>
#include <pbd_msgs/PbdSceneGraph.h>


namespace ASR
{

class asrSceneFinderPSM
{
private:
    std::string SceneModelFileName;

    /**
     * Interface to private ros node namespace.
     */
    ros::NodeHandle mNodeHandle;

    /**
      * A transformer for objects into the target coordinate frame.
      */
    ProbabilisticSceneRecognition::ObjectTransformation mObjectTransform;


    /**
    * The model is responsible for loading the scene model from file, collect and manage the evidence and do the inference.
    */
    boost::shared_ptr<ProbabilisticSceneRecognition::SceneModelDescription> mModel;

    /**
      Contains all hypotheses which were calculated.
      A hypothesis contains the object type and the expected pose.
      */
    std::vector<AttributedPoint> hypotheses;

    /**
      * A buffer for storing evidences.
      * evidences are object messages.
      */
    std::queue<boost::shared_ptr<pbd_msgs::PbdObject> > mEvidenceBuffer;

    const boost::shared_ptr<pbd_msgs::PbdObject> convertPdbObjectMsg(const pbd_msgs::PbdObject::ConstPtr& msg);

    void updateHypotheses();

    /**
     * Extract PbdSceneGraph messages from all rosbag files given as CLI parameters.
     *
     * @param pInputBagFilenames A list of the bag files that contain the learning data.
     */
    void readLearnerInputBags(XmlRpc::XmlRpcValue pInputBagFilenames);

    /**
     * Open rosbag file and extract PbdSceneGraph messages on input topic (which has been set before).
    *
    * @param pPbdSceneGraphsBagPath Path of file to be parsed for PbdSceneGraph messages.
     */
    void extractPbdSceneGraphsFromBag(const std::string& pPbdSceneGraphsBagPath);

    /**
    * Collects scene examples in form of PbdSceneGraph messages and forwards them to the visualization.
     *
     * @param pSceneGraph Preprocessed observations that describe the objects in a scene over time.
    */
    void newSceneGraphCallback(const boost::shared_ptr<const pbd_msgs::PbdSceneGraph>& pSceneGraph);

    /**
    * A callback handler listening to preprocessed observations that describe the objects in a scene over time
    */
    ros::Subscriber mSceneGraphListener;

    /**
    * A buffer for storing scene graphs.
    */
    std::queue<boost::shared_ptr<const pbd_msgs::PbdSceneGraph> > mSceneGraphBuffer;


public:

    asrSceneFinderPSM();

    /**
      Is called when an object is recognized.
      Checks if the recognition is valid and calculates object hypothesis.
      */
    void onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg);

    std::vector<AttributedPoint> getHypotheses()
    {
        return hypotheses;
    }

};

typedef boost::shared_ptr<asrSceneFinderPSM> asrSceneFinderAlgorithmPSMPtr;


}
