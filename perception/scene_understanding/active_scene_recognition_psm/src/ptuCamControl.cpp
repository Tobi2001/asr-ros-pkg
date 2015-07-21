#include "ptuCamControl.h"

#include <unistd.h>
#include <cfloat>
#include <iostream>
#include <sstream>
#include <fstream>

#include <boost/filesystem.hpp>

#define TOPIC_CAMERA_INFO "stereo/left/camera_info"
#define TOPIC_STATE_CMD "state_cmd"

#define SERVICE_PTU_SETTINGS "update_speed_control"

namespace ASR {

ptuCamControl::ptuCamControl(float window_adjustment) :
    dpp_x(0.0287),
    dpp_y(0.0253),
    isInitialized(false)
{
    lastView = View();
    nextView = View();

    xMin = 0;
    xMax = 0;
    yMin = 0;
    yMax = 0;

    //Some handles to access different parameter sets (from different ros nodes) in parameter server.
    tracker = ros::NodeHandle("ptu_tracker");
    ptu = ros::NodeHandle("ptu_driver");

    this->window_adjustment = window_adjustment;

    ptu.setParam("speed_control", false);
    settingsClient.call(empty);

    tracker.getParam("camera_frame", cameraFrame);
    tracker.getParam("ptu_base_frame", ptuFrame);
    tracker.getParam("dpp_x", dpp_x);
    tracker.getParam("dpp_y", dpp_y);

    ptu.getParam("pan_min_angle", xMin);
    ptu.getParam("pan_max_angle", xMax);
    ptu.getParam("tilt_min_angle", yMin);
    ptu.getParam("tilt_max_angle", yMax);

    ROS_INFO("PTU Working Space: xMin, xMax (%.2f, %.2f); yMin, yMax (%.2f, %.2f)", xMin,xMax,yMin,yMax);

    //Interface to PTU
    settingsClient = ptu.serviceClient<std_srvs::Empty>(SERVICE_PTU_SETTINGS);
    stateCmdPub = ptu.advertise<sensor_msgs::JointState>(TOPIC_STATE_CMD, 1);

    //Interface to camera.
    cameraInfoSub = others.subscribe<sensor_msgs::CameraInfo>(TOPIC_CAMERA_INFO, 1, &ptuCamControl::onCameraInfoMessage, this);
}


void ptuCamControl::onCameraInfoMessage(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  ROS_DEBUG("Initialising camera with CameraInfo msg.\n");

  cameraInfoSub.shutdown();

  Eigen::Vector2f dpp;
  dpp << dpp_x, dpp_y;
  Eigen::Vector2f imageDims;
  imageDims << msg->width, msg->height;

  //Calculate field of view (in degrees) of PTU based von degrees per pixel and image size. In fact this is the size within image area boundaries calculated below.
  ROS_INFO("Image size and how much percent of it is used: (%d,%d)-(%.2f). dpp_x(%.2f), dpp_y(%.2f)\n", msg->width, msg->height, window_adjustment, dpp_x, dpp_y);
  cameraFoV << imageDims.x() * dpp.x(), imageDims.y() * dpp.y();
  cameraFoV << cameraFoV.x() * window_adjustment, cameraFoV.y() * window_adjustment;
  ROS_INFO("This given a camFoV in deg: (%.2f,%.2f).\n", cameraFoV.x(), cameraFoV.y());

  //Calculate in which image area boundary (for an arbitrary camera view) points are considered while processing.
  RoIMin.x() = imageDims.x() * ((1-window_adjustment) / 2);
  RoIMin.y() = imageDims.y() * ((1-window_adjustment) / 2);
  RoIMax.x() = imageDims.x() * (1 - (1-window_adjustment) / 2);
  RoIMax.y() = imageDims.y() * (1 - (1-window_adjustment) / 2);

  ROS_INFO("ROI extent in x: (%d,%d), in y: (%d,%d).\n", RoIMin.x(), RoIMax.x(), RoIMin.y(), RoIMax.y());

  ROS_DEBUG("Camera Initialization finished.\n");
  settingsClient.waitForExistence();

  isInitialized = true;
}



/**
  Checks if a given pan_tilt configuration is inside the ptu
  working space.
  @param pan_tilt the new configuration in degrees
  */
bool ptuCamControl::checkIfViewValid(Eigen::Vector2f pan_tilt)
{
    float x = pan_tilt.x();
    float y = pan_tilt.y();

    return (         x > xMin
                &&   x < xMax
                &&   y > yMin
                &&   y < yMax);
}

/*
 private Method. It is called in updatePTU.
 Sets a new pan and tilt angle and rotates the ptu.
 */
void ptuCamControl::sendOrientation2PTU(Eigen::Vector2f pan_tilt)
{
    ROS_ASSERT(checkIfViewValid(pan_tilt));


    bool speedControl = true;
    ptu.getParam("speed_control", speedControl);
    if (speedControl)
    {
        ptu.setParam("speed_control", false);
        settingsClient.call(empty);
    }

    sensor_msgs::JointState pantiltState;
    pantiltState.header.stamp = ros::Time::now();

    pantiltState.name.push_back("pan");
    pantiltState.name.push_back("tilt");

    pantiltState.position.push_back(pan_tilt.x());
    pantiltState.position.push_back(pan_tilt.y());

    pantiltState.velocity.push_back(pan_tilt.x());
    pantiltState.velocity.push_back(pan_tilt.y());

    stateCmdPub.publish(pantiltState);
}

/**
  Sets a new View. The camera will move to that view. updatePTU has to be called.
  The camera/ptu will move to that configuration if it is inside
  the ptu working space.
  */
void ptuCamControl::setNextView(View nextView)
{
    setNextView(nextView.center);
}

/**
  Sets a new View. The camera will move to that view. updatePTU has to be called.
  The camera/ptu will move to that configuration if it is inside
  the ptu working space.
  @param pan_tilt the new configuration in degrees
  */
void ptuCamControl::setNextView(Eigen::Vector2f pan_tilt)
{
    if (!checkIfViewValid(pan_tilt))
    {
        ROS_INFO("pan_tilt not valid. Check PTU boundaries:\n pan = %.2f, [%.2f]< pan <[%.2f]\n tilt= %.2f, [%.2f]< tilt <[%.2f]",
                 pan_tilt.x(), xMin, xMax , pan_tilt.y(), yMin, yMax);
        return;
    }
    nextView.center = pan_tilt;
    nextView.valid = true;
    nextView.fov = getCameraFOV();
}


/**
  Updates the camera/ptu.
  If the nextView is reached the camera/ptu stops.
  If not it will move to the next configuration which is
  set by calling setNextview(..)
  */
void ptuCamControl::updatePTU()
{
    float next_x = nextView.center.x();
    float next_y = nextView.center.y();
    float last_x = lastView.center.x();
    float last_y = lastView.center.y();

    if (reachedDesiredPosition() && isInitialized)
    {
        lastView = nextView;
    }

    if(fabs(next_x - last_x) < 0.1 && fabs(next_y - last_y) < 0.1)
    {
        // nextView == lastView
        // stop moving
        ROS_DEBUG("Reached desired Position (%.2f, %.2f)", lastView.center.x(), lastView.center.y());
    } else
    {
        // nextView != lastView
        // update ptu position
        ROS_DEBUG("Moving to desired Position (%.2f, %.2f)", nextView.center.x(), nextView.center.y());
        //sendOrientation2PTU(nextView.center);
    }
}

/**
  Returns whether the camera/ptu stopped.
  @return true if the ptu stands still, returns false if it moves
  */
bool ptuCamControl::reachedDesiredPosition()
{
    bool stopped = false;
    ptu.getParam("reached_desired_position", stopped);
    return stopped;
}


/**
  Calculates the sphere coordinates of a point.
  @param the point cartesian coordinates
  @return the point in sphere coordinates
  */
Eigen::Vector2f ptuCamControl::pointToSphereCoords(Eigen::Vector3f p)
{
    /*
    if((sqrt(p.x() * p.x() + p.y() * p.y())) == 0.0)
        return Eigen::Vector2f(0,0);


    double panAngle = (p.x() < 0)? 1 : -1;
    panAngle *= acos(-p.y() / (sqrt(p.x() * p.x() + p.y() * p.y()))) * RAD_TO_DEG;

    double tiltAngle = (p.z() > 0)? -1 : 1;
    tiltAngle *= acos(-p.y() / (sqrt(p.y() * p.y() + p.z() * p.z()))) * RAD_TO_DEG;

    Eigen::Vector2f sphereCoords(panAngle, tiltAngle);
    ROS_INFO("Speher coords: (%.2f, %0.2f)", panAngle, tiltAngle);*/
    return pointToSphereCoords(p.x(), p.y(), p.z());
}

Eigen::Vector2f ptuCamControl::pointToSphereCoords(float x, float y, float z)
{
    double r = sqrt(x*x + y*y + z*z);
    double phi = asin(z/r);
    double tan = atan2(y,x);

    //ROS_INFO("Sphere coords: (%.2f, %0.2f)", phi * RAD_TO_DEG, tan*RAD_TO_DEG);
    return Eigen::Vector2f(phi, tan);
}



}

