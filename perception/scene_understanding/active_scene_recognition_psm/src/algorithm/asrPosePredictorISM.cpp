#include "asrPosePredictorISM.h"

namespace ASR
{


asrPosePredictorISM::asrPosePredictorISM()
    : asrPosePredictor()
{
    algorithm = asrSceneFinderISMPtr(new asrSceneFinderISM());
}

void asrPosePredictorISM::onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg)
{
    algorithm->onObjectMessage(msg);
}

std::vector<AttributedPoint> asrPosePredictorISM::updateHypotheses()
{
    if(algorithm->calculationFinished())
    {
        hypotheses = algorithm->getHypotheses();
    }
    return hypotheses;
}









}
