#include "asrStateMachine.h"

#include <unistd.h>
#include <cfloat>
#include <iostream>
#include <sstream>
#include <fstream>

#include <boost/filesystem.hpp>


#define TOPIC_OBJECTS "/stereo/objects"
#define PARAM_IDLE "is_idle"

#define FILTERTHRESHOLD 0.0001

namespace ASR {

  asrStateMachine::asrStateMachine(ros::NodeHandle& n) :
      priv(n)
  {
    ROS_DEBUG("Booting asr_state_machine.\n");
    ROS_DEBUG("Switched to STATE_NONE.\n");
    state = STATE_NONE;

    //Some handles to access different parameter sets (from different ros nodes) in parameter server.
    priv = ros::NodeHandle("~");

    if(!priv.getParam("window_adjustment", window_adjustment))
          window_adjustment = 0.75;

    if(!priv.getParam("searching_timer", searching_timer))
          searching_timer = 5;

    searchingTimer = n.createTimer(ros::Duration(searching_timer), &asrStateMachine::searchingTimerCallback, this);
    searchingTimer.stop();

    // initialize the ptu and the camera
    ptuManager = ptuManagerPtr(new ptuCamControl(window_adjustment));

    //XXX needed for vis
    //stateSub = ptu.subscribe("state", 1, &asrStateMachine::onStateMessage, this);
    changeViewCounter = 0;

    ROS_DEBUG("Booting up asr state machine finished.\n");
  }


  void asrStateMachine::onStateMessage(const sensor_msgs::JointState::ConstPtr& msg)
  {
    if (ptuManager->getCameraFOV().x() < 1 || ptuManager->getCameraFOV().y() < 1) return;
  }



  /**
    Switches the current state to the new given state.
  */
  void asrStateMachine::switchToState(int newState)
  {
      ROS_ASSERT(newState > 0 && newState <= 6);

      std::string states[] = {"STATE_NONE", "STATE_READY",
                              "STATE_WAITING_FOR_PTU_TO_STOP", "STATE_SEARCHING",
                              "STATE_NEXT_SCENE_HYP", "STATE_INITIAL_SEARCH", "STATE_INIT_ALL"};

      ROS_DEBUG("Switched to %s\n", states[newState].c_str());
      state = newState;
  }

  /**
    Precondition: None
    Next State: STATE_INIT_ALL
    On STATE_NONE nothing happens until the camera/ptu is ready.
    The camera initializes asynchron.
  */
  void asrStateMachine::onStateNone()
  {
      // wait until the camera is initialized
      if(ptuManager->isPTUReady())
          switchToState(STATE_INIT_ALL);
      else
          ROS_DEBUG("Waiting for camera info msg...");
  }

  /**
    Precondition: camera/ptu has to be initialized
    Next State: STATE_INITIAL_SEARCH
    This state initializes other modules and it is guaranteed that the camera/ptu
    is fully initilized.
  */
  void asrStateMachine::onStateInitAll()
  {
      // initialize spiral search
      initialSearcher = SpiralSearcherPtr(new SpiralSearcher(ptuManager->getCameraFOV()));

      // Move the ptu to the initial position
      View nextView = initialSearcher->next();
      nextView.valid = ptuManager->checkIfViewValid(nextView.center);

      ROS_INFO("First orientation, initial search moves to: (%.2f,%.2f).",nextView.center.x(),nextView.center.y());
      ptuManager->setNextView(nextView.center);


      // initialize visualizer
      mVisualizer = asrVisualizerPtr(new asrVisualizer());
      mVisualizer->initAnimatedPlot("Object Positions in PTU Space",
                                    "Pan Angle [deg]",
                                    "Tilt Angle [deg]",
                                    std::make_pair(-60, 60),
                                    std::make_pair(-60, 60),
                                    std::make_pair(10, 10 ));

      mVisualizer->addPointToFoundBuffer(ptuManager->pointToSphereCoords(0.5,0,1));

      //Testing
      {
          sGrid = simpleGridPtr(new simpleGrid(Eigen::Vector2f(-60,-60), Eigen::Vector2f(60,60), ptuManager->getCameraFOV()));
      }
      // /Testing


      // initialize the scenefinder
      objectsSub = priv.subscribe(TOPIC_OBJECTS, 10, &asrStateMachine::onObjectMessage, this);
      sceneFinder = asrPosePredictorISMPtr(new asrPosePredictorISM());

      // initialization finished. Now start searching.
      searchingTimer.start();
      switchToState(STATE_INITIAL_SEARCH);
  }

  /**
    This method is called every "searching_timer" seconds.
    It sets the next view for the camrea/ptu.
    */
  void asrStateMachine::searchingTimerCallback(const ros::TimerEvent &)
  {
      // Tseting
      {

          for(auto p : mVisualizer->mUnfoundBuffer)
          {
              sGrid->addPoint(p.first, p.second);
          }


          Eigen::Vector2f nextView = sGrid->getCenterOfNextView();

          ROS_DEBUG("Next View: (%.2f, %.2f)", nextView.x(), nextView.y() );

          ptuManager->setNextView(nextView);


      }
      // / Testing


      // Set the next View to move to
      //ptuManager->setNextView(initialSearcher->next());

      // For evalutation count the # of needed views
      changeViewCounter++;
  }

  /**
    Precondition: camera/ptu and spiral search initialized
    Next state: STATE_INITIAL_SEARCH or STATE_NEXT_SCENE_HYPO
    Moves the camera/ptu without scene knowledge to find the first object.
    It loops until a object has been found. If a object has been found
    the machine switches to state STATE_NEXT_SCENE_HYPO.
    */
  void asrStateMachine::onStateInitialSearch()
  {
      // the nextView is updated every "searching_timer" seconds
      // this happens in searchingTimerCallback      

      // update the visualization
      mVisualizer->updateGnuplotData(ptuManager->getNextView(), ptuManager->getLastView());
      mVisualizer->sendPlotToGnuplot();

      // move the ptu
      ptuManager->updatePTU();
  }

  /**
    Precondition: IntilialSearch or informedSearch
    Next state: STATE_SEARCHING
    A object has been recognized.
    The scene hypothesis are updated and calculated.
    This state waits until all hypothesis are calculated.
    After that it starts searching again.
    */
  void asrStateMachine::onStateNextSceneHypo()
  {
      // wait for asrSceneFinder to find nextView
      //if(!sceneFinder->calculationFinished()) return;

      // set nextView
      // toDo call searchingTimer.start();
      // toDo switchToState(STATE_SEARCHING);
  }




  /**
    Do the functionality of the current state.
    This function is called in the frequency loop_rate (currently 1Hz)
    */
  void asrStateMachine::spinAndDraw()
  {
    ros::spinOnce();

    // Switch the current state and call the ralated functionality
    switch(state)
    {
        case(STATE_NONE):
        {
            onStateNone();
        } break;
        case(STATE_INIT_ALL):
        {
            onStateInitAll();
        } break;
        case (STATE_INITIAL_SEARCH):
        {
            onStateInitialSearch();
        } break;
        case (STATE_NEXT_SCENE_HYP):
        {
            onStateNextSceneHypo();
        } break;
        default:
        {
            ROS_DEBUG("Something went wrong. State Machine is in a undefinded state..");
        }
    }
  }


  void asrStateMachine::onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg)
  {
      // a object message is only of interest if
      // 1. the correct view is reached and
      // 2. we are looking for objects
      if(ptuManager->reachedDesiredPosition() && (state == STATE_INITIAL_SEARCH || state == STATE_SEARCHING))
      {
          searchingTimer.stop();
          switchToState(STATE_NEXT_SCENE_HYP);
          sceneFinder->onObjectMessage(msg);
      }
  }

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_scene_recognition_psm");
  ros::NodeHandle n("~");
  ASR::asrStateMachine* stateMachine = new ASR::asrStateMachine(n);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    stateMachine->spinAndDraw();
    loop_rate.sleep();
  }

  return 0;
}
