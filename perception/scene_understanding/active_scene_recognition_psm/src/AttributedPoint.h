#pragma once

#include <geometry_msgs/PoseWithCovariance.h>

namespace ASR
{


struct AttributedPoint {
    std::string objectType;
    geometry_msgs::Pose pose;

    AttributedPoint(std::string type, geometry_msgs::Pose  p)
    {
        objectType = type;
        pose=p;
    }
};

}
