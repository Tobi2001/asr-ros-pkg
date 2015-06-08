// Global includes
#include <cstdlib>

// Package includes
#include <ros/ros.h>

#include <pbd_msgs/pbd_computation_graph.hpp>

// Local includes
#include "learner/SceneLearningEngine.h"

using namespace ProbabilisticSceneRecognition;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "js_probabilistic_scene_learner");
  
  // Declaration of the learner for the scene model.
  SceneLearningEngine* learner;

  // Check for wrong configurations and similar.
  try {
    learner = new SceneLearningEngine(pbd_msgs::Topics::SCENE_GRAPHS);

  } catch(std::exception& exception){
    std::cerr << exception.what() << std::endl;
    std::exit(1);
  }

  // Get all PbdSceneGraphs from rosbag files passed as ros parameters before trying to get other messages from the listeners of the node.
  learner->readLearnerInputBags();

  // Check for errors with the resulting scene model
  try {       
    // Calculate parameters of scene model based on scene graphs from rosbag input.
    learner->generateSceneModel();
    
    // Dump model to file and plot its distributions.
    learner->saveSceneModel();
  } catch(std::exception& exception) {
    std::cerr << exception.what() << std::endl;
  }
  
  // Specifies the updates per second.
  ros::Rate rate(30);
  
  // Run main loop until termination.
  while(ros::ok())
  {
    // Visualize the model.
    learner->visualizeSceneModel();
    
    // Sleep for the given time in seconds.
    rate.sleep();
  }
  
  // Get rid of learner object.
  delete learner;

  // Stating that program has run correctly.
  return EXIT_SUCCESS;
}

