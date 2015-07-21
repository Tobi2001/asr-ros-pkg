#include "helper/ObjectTransformation.h"

namespace ProbabilisticSceneRecognition {
 
  ObjectTransformation::ObjectTransformation()
  : mTfListener(ros::Duration(5184000.0))
  {
  }
  
  ObjectTransformation::~ObjectTransformation()
  {
  }
  
  void ObjectTransformation::setBaseFrame(std::string pBaseFrame)
  {
    mBaseFrame = pBaseFrame;
  }
  
  void ObjectTransformation::transform(const boost::shared_ptr<pbd_msgs::PbdObject>& pObject)
  {
    // Create everything required for the transformation
    geometry_msgs::PoseStamped input, output;
    input.header = pObject->header;
    input.pose = pObject->poseEstimation.pose;
    
    // If no transformation from source to target frame possible, drop object.
    if(!mTfListener.waitForTransform(mBaseFrame, input.header.frame_id, pObject->header.stamp, ros::Duration(1.0)))
      throw std::runtime_error("Unable to resolve transformation in target coordinate frame.");
    
    // Do the transformation.
    mTfListener.transformPose(mBaseFrame, input, output);
    
    // Write the results back.
    pObject->header = output.header;
    pObject->poseEstimation.pose = output.pose;
  }
  
}