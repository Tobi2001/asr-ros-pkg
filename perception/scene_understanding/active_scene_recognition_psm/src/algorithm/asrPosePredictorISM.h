#pragma once

#include "../asrPosePredictor.h"
#include "asrSceneFinderISM.h"

namespace ASR
{

class asrPosePredictorISM
        : public asrPosePredictor
{
public:
    void onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg);
    std::vector<AttributedPoint> updateHypotheses();
    asrPosePredictorISM();

private:
    asrSceneFinderISMPtr algorithm;

};

typedef boost::shared_ptr<asrPosePredictorISM> asrPosePredictorISMPtr;


}


