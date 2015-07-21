// Global includes
#include <stdio.h>

// Package includes
#include <ros/ros.h>

// Local includes
#include "inference/SceneInferenceEngine.h"

using namespace ProbabilisticSceneRecognition;

/**
 * Inference engine main program.
 * Sets up a ros node running the inference engine.
 */
int main(int argc, char* argv[])
{
  // Initialize ros node.
  ros::init(argc, argv, "js_probabilistic_scene_inference_engine");
  
  // Create inference engine.
  SceneInferenceEngine ie;
  
  // Specifies the updates per second.
  ros::Rate rate(30);
  
  // Run main loop until termination.
  while(ros::ok()) {
    
    // Visualize the probabilistic scene model.
    ie.update();

    // Longing for callbacks.
    ros::spinOnce();
    
    // Sleep for the given time in seconds.
    rate.sleep();
  }
  return 0;
}
