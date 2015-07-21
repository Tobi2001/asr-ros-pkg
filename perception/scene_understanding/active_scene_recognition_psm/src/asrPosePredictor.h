#pragma once

#include <pbd_msgs/PbdObject.h>
#include "AttributedPoint.h"

namespace ASR
{

class asrPosePredictor
{
public:

    virtual void onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg) = 0;
    virtual std::vector<AttributedPoint> updateHypotheses() = 0;


protected:
    std::vector<AttributedPoint> hypotheses;

};



}
