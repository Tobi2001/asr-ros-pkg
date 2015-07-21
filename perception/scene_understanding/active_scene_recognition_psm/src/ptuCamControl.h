#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <Eigen/Core>

#include "View.h"

namespace ASR
{
#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (1 / RAD_TO_DEG)

  // PTU and Camera Controller. Controls the ptu movement and communication
  class ptuCamControl
  {
  private :
      ros::NodeHandle ptu;
      ros::NodeHandle tracker;
      ros::NodeHandle others;

      Eigen::Vector2f cameraFoV;
      //image area boundary
      Eigen::Vector2i RoIMin;
      Eigen::Vector2i RoIMax;

      std::string cameraFrame;
      std::string ptuFrame;

      ros::Publisher stateCmdPub;

      ros::Subscriber cameraInfoSub;
      ros::Subscriber stateSub;

      ros::ServiceClient settingsClient;
      std_srvs::Empty empty;

      //Boundaries of working space of PTU.
      double xMin, xMax, yMin, yMax;

      //Transform from pixels to degrees
      double dpp_x;
      double dpp_y;

      // how much % of the actual FoV the PTU is assumed to see
      float window_adjustment;

      View lastView;
      View nextView;

      bool isInitialized;

      // Sets a new pan and tilt angle and rotates the ptu
      void sendOrientation2PTU(Eigen::Vector2f pan_tilt);

  public:
      ptuCamControl(float window_adjustment);

      // Set field of view of camera in our model based on real camera properties.
      void onCameraInfoMessage(const sensor_msgs::CameraInfo::ConstPtr& msg);

      // Sets a new View. The camera will move to that view. updatePTU has to be called.
      void setNextView(Eigen::Vector2f pan_tilt);
      void setNextView(View nextView);

      // Moves the PTU to the nextView. It stoppes if the lastView is near the nextView.
      void updatePTU();

      // Returns true if the ptu stands still, returns false if it moves
      bool reachedDesiredPosition();

      // Checks if the given center of view is inside the camera operating space
      bool checkIfViewValid(Eigen::Vector2f pan_tilt);

      // Calculates the sphere coordinates of a point.
      Eigen::Vector2f pointToSphereCoords(Eigen::Vector3f p);
      Eigen::Vector2f pointToSphereCoords(float x, float y, float z);

  public:
      // Getters
      Eigen::Vector2f getCameraFOV() {return cameraFoV;}
      double getDPP_X() {return dpp_x;}
      double getDPP_Y() {return dpp_y;}
      float getWindowAdjustment() {return window_adjustment;}
      View getLastView() {return lastView;}
      View getNextView() {return nextView;}
      bool isPTUReady() {return isInitialized;}
      Eigen::Vector2i getRoIMin() {return RoIMin;}
      Eigen::Vector2i getRoIMax() {return RoIMax;}
  };


  typedef boost::shared_ptr<ptuCamControl> ptuManagerPtr;
}
