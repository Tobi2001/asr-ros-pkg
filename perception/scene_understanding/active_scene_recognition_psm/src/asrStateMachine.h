#ifndef OBJECTSEARCHER_H_
#define OBJECTSEARCHER_H_

#include <ros/ros.h>
#include <limits.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ptuCamControl.h"
#include "spiralSearcher.h"
#include "View.h"
#include "asrVisualizer.h"
#include "algorithm/asrPosePredictorISM.h"
#include "algorithm/asrPosePredictorPSM.h"
#include "grid.h"

#define WAITING_TIMER 0.1

#define STATE_NONE 0
#define STATE_READY 1 //all data initialized
#define STATE_WAITING_FOR_PTU_TO_STOP 2
#define STATE_SEARCHING 3
#define STATE_NEXT_SCENE_HYP 4
#define STATE_INITIAL_SEARCH 5
#define STATE_INIT_ALL 6

#define INIT_CAMERA 0x1
#define INIT_ALL 0x1

namespace ASR {
    //State machine controlling different search algorithms.
    class asrStateMachine {

    public:
        asrStateMachine(ros::NodeHandle& n);

        void spinAndDraw();

    private:
        // Spiral Searcher
        SpiralSearcherPtr initialSearcher;

        // Visualization
        asrVisualizerPtr mVisualizer;

        // Testing
        unsigned int changeViewCounter;
        simpleGridPtr sGrid;
        // /Testing

        ros::Publisher mDataPub;

        // ObjectSubscriber
        ros::Subscriber objectsSub;

        // the ptu controller
        ptuManagerPtr ptuManager;
        // the scene finder
        asrPosePredictorISMPtr sceneFinder;


        void onStateMessage(const sensor_msgs::JointState::ConstPtr& msg);

        //Used to access parameters in parameter server of different and this node.
        ros::NodeHandle priv;

        char state;

        //params
        double searching_timer;
        double window_adjustment;

        ros::Timer searchingTimer;

    private:
        void onObjectMessage(const pbd_msgs::PbdObject::ConstPtr& msg);
        void searchingTimerCallback(const ros::TimerEvent&);

        //state methods
        void switchToState(int newState);
        void onStateNone();
        void onStateInitAll();
        void onStateInitialSearch();
        void onStateNextSceneHypo();


    };
}

#endif
