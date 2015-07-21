#include "asrPosePredictorPSM.h"


namespace ASR
{

asrPosePredictorPSM::asrPosePredictorPSM()
{
    algorithm = asrSceneFinderAlgorithmPSMPtr(new asrSceneFinderPSM());
}

void asrPosePredictorPSM::onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg)
{
    algorithm->onObjectMessage(msg);
}

std::vector<AttributedPoint> asrPosePredictorPSM::updateHypotheses()
{
    return algorithm->getHypotheses();
}

}
