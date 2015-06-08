// Global includes
#include <stdio.h>

// Package includes
#include <ros/ros.h>
#include <ros/console.h>

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
  
  // Create and execute inference engine.
  SceneInferenceEngine ie;
  ie.executeInStackMode();

  // Longing for callbacks.
  ros::spinOnce();
  
  return 0;
}
