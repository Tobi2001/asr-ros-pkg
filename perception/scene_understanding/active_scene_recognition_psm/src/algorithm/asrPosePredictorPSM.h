#pragma once

#include "../asrPosePredictor.h"
#include "asrSceneFinderPSM.h"

namespace ASR
{


class asrPosePredictorPSM : public asrPosePredictor
{
public:
    asrPosePredictorPSM();


    void onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg);

    std::vector<AttributedPoint> updateHypotheses();

private:
    asrSceneFinderAlgorithmPSMPtr algorithm;

};


}
