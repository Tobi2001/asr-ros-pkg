#include <ros/ros.h>
#include "spiralSearcher.h"

namespace ASR {

  SpiralSearcher::SpiralSearcher(Eigen::Vector2f pStepSizeInAxes) :
      round(1),
      step(0),
      dir(0)
  {
    ros::NodeHandle n("~");
    
    double pan, tilt;

    n.getParam("abortionScore", mAbortionScore);
    n.getParam("starting_orientation_pan", pan);
    n.getParam("starting_orientation_tilt", tilt);

    mFutureOrientation = Eigen::Vector2f(pan, tilt);

    //Pan and tilt steps.
    mPossibleSteps.push_back(Eigen::Vector2f(0, -pStepSizeInAxes.y()));
    mPossibleSteps.push_back(Eigen::Vector2f(pStepSizeInAxes.x(), 0));
    mPossibleSteps.push_back(Eigen::Vector2f(0, pStepSizeInAxes.y()));
    mPossibleSteps.push_back(Eigen::Vector2f(-pStepSizeInAxes.x(), 0));

    mTargetView.fov = pStepSizeInAxes;

  }

  View SpiralSearcher::next()
  {
    mTargetView.score = mAbortionScore + 1.0f;
    mTargetView.center = mFutureOrientation;

    //One step forward in current direction
    mFutureOrientation += mPossibleSteps[dir % mPossibleSteps.size()];

    step++;

    //Change direction when segment length is equal to step size in one direction. 
    if (step == round) {
      step = 0;
      dir++;
    }
    //Increase spiral radius "by one" every two spiral segments resp. segment length.
    if (dir % 2 == 0 && step == 0) {
      round++;
    }

    return mTargetView;
  }

}
