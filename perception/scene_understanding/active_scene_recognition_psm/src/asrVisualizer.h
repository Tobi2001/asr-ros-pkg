#pragma once

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include "gnuplot-iostream.h"
#include "View.h"

#include <string>
#include <set>
#include <utility>

#include <Eigen/Core>


namespace ASR
{

    class asrVisualizer
    {
    public:
        asrVisualizer();

        // Initializes the GnuPlot
        void initAnimatedPlot(const std::string& pPlotTitle,
                              const std::string& pXLabel,
                              const std::string& pYLabel,
                              const std::pair<float, float>& pXRange,
                              const std::pair<float, float>& pYRange,
                              const std::pair<float, float>& pDelta);


        // Updates the data that is visualized with the gnuplot.
        void updateGnuplotData(View nextView, View lastView);

        // Sends the data to the gnuplot and draws the window which contains the plot.
        void sendPlotToGnuplot();

        // Add points to the buffers
        void addPointToFoundBuffer(Eigen::Vector2f point);
        void addPointToUnfoundBuffer(Eigen::Vector2f point);


    private:

        //Interface with which configurations or data is sent to gnuplot.
        boost::shared_ptr<Gnuplot> mGnuplotHandler;

        View nextView;
        View lastView;

        //buffers for visualization points
    public:
        std::vector<std::pair<double, double> > mUnfoundBuffer;
        std::vector<std::pair<double, double> > mFoundBuffer;
        std::vector<std::pair<float, float> > mVisitedViews;

    };


    typedef boost::shared_ptr<asrVisualizer> asrVisualizerPtr;

}
