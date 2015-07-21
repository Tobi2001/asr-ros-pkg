#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "View.h"

namespace ASR {

  //Spiral search as long as no object is found.
  class SpiralSearcher
  {

  public:
    //Set sizes of atomar steps performed during spiral search and starting orientation for spiral search. Unit is degrees.
    SpiralSearcher(Eigen::Vector2f pStepSizeInAxes);
    //Calculate next step in spiral search based on current position (in relation to )
    View next();

  private:
    //Step size to alter camera orientation in pan and tilt.
    std::vector<Eigen::Vector2f> mPossibleSteps;
    //Orientation that PTU will seek after calling next().
    Eigen::Vector2f mFutureOrientation;
    int round;
    int step;
    unsigned int dir;

    double mAbortionScore;

    View mTargetView;

  };

  typedef boost::shared_ptr<SpiralSearcher> SpiralSearcherPtr;

}
