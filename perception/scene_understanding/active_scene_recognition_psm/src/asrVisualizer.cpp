#include "asrVisualizer.h"

namespace ASR
{

asrVisualizer::asrVisualizer()
{
    nextView = View();
    lastView = View();

    /*
      Add some test data to the buffers
      */
    mFoundBuffer.push_back(std::make_pair(12, 17));
    mFoundBuffer.push_back(std::make_pair(9, 39));

    mUnfoundBuffer.push_back(std::make_pair(0, 0));
    mUnfoundBuffer.push_back(std::make_pair(5, 6));
    mUnfoundBuffer.push_back(std::make_pair(9, 6));
    mUnfoundBuffer.push_back(std::make_pair(-27, 7));
    mUnfoundBuffer.push_back(std::make_pair(-27, 6));
    mUnfoundBuffer.push_back(std::make_pair(-27, 6));

    mUnfoundBuffer.push_back(std::make_pair(-27, -30));

}

/**
  Adds a new point to the list of found points
  */
void asrVisualizer::addPointToFoundBuffer(Eigen::Vector2f point)
{
    mFoundBuffer.push_back(std::make_pair(point.x(), point.y()));
}

/**
  Adds a new point to the list of unfound points
  */
void asrVisualizer::addPointToUnfoundBuffer(Eigen::Vector2f point)
{
    mUnfoundBuffer.push_back(std::make_pair(point.x(), point.y()));
}

/**
  Initializes the GnuPlot.
  @param pPlotTitle - the name of the plot
  @param pxLabel - label of the x-axis
  @param pyLabel - label of the y-axis
  @param pXRange - x-axis range (min, max)
  @param pYRange - y-axis range (min, max)
  @param pDelta - scale of the helper lines
  */
void asrVisualizer::initAnimatedPlot(const std::string& pPlotTitle,
                                     const std::string& pXLabel, const std::string& pYLabel,
                                     const std::pair<float, float>& pXRange, const std::pair<float, float>& pYRange,
                                     const std::pair<float, float>& pDelta) {

    //Create a clean interface to gnuplot.
    mGnuplotHandler.reset(new Gnuplot);

    //Empty buffers with data for gnuplot.
    //mUnfoundBuffer.clear();
    //mFoundBuffer.clear();

    //Set bar chart title
    *(mGnuplotHandler) << "set title \"" << pPlotTitle << "\"\n";

    //Unit length in plot of both x and y axis are equal.
    *(mGnuplotHandler) << "set size ratio -1\n";

    //Style for points
    *(mGnuplotHandler) << "set border linewidth 0.5\n";
    *(mGnuplotHandler) << "set pointsize 1\n";
    *(mGnuplotHandler) << "set style line 1 lc rgb '#00008b' pt 5\n";
    *(mGnuplotHandler) << "set style line 2 lc rgb '#00ced1' pt 7\n";

    //Set labels for axes
    *(mGnuplotHandler) << "set xlabel \"" << pXLabel << "\"\n";
    *(mGnuplotHandler) << "set ylabel \"" << pYLabel << "\"\n";

    //Set range in both x and y direction
    *(mGnuplotHandler) << "set xrange [" << pXRange.second <<  ":" << pXRange.first << "]\n";
    *(mGnuplotHandler) << "set yrange [" << pYRange.first <<  ":" << pYRange.second << "]\n";

    //Ask for a grid for highres ticks
    *(mGnuplotHandler) << "set grid x2tics lc rgb \"#bbbbbb\"\n";
    *(mGnuplotHandler) << "set grid y2tics lc rgb \"#bbbbbb\"\n";

    //Lowres ticks
    *(mGnuplotHandler) << "set xtics " << pXRange.first << "," << pDelta.first * 4 << "," << pXRange.second <<"rotate\n";
    *(mGnuplotHandler) << "set ytics " << pYRange.first << "," << pDelta.second * 4 << "," << pYRange.second <<"\n";

    //Highres tics
    *(mGnuplotHandler) << "set x2tics " << pXRange.first << "," << pDelta.first << "," << pXRange.second <<" scale 0\n";
    *(mGnuplotHandler) << "set y2tics " << pYRange.first << "," << pDelta.second << "," << pYRange.second <<" scale 0\n";

    //Highres ticks with labels please
    *(mGnuplotHandler) << "set format y2 \"\"\n";
    *(mGnuplotHandler) << "set format x2 \"\"\n";
}

/**
  Updates the data that is visualized with the gnuplot.
  @param nextView - sets a new nextView
  @param lastView - sets a new lastView
  */
void asrVisualizer::updateGnuplotData(View nextView, View lastView)
{
    ROS_ASSERT(nextView);
    ROS_ASSERT(lastView);

    this->nextView = nextView;
    this->lastView = lastView;

    // add last view center to the visited points. Ignore the initial point (0.0, 0.0)
    if(mVisitedViews.size() != 0 || lastView.center.x() != 0.0 || lastView.center.y() != 0.0)
        this->mVisitedViews.push_back(std::make_pair(lastView.center.x(), lastView.center.y()));
}

/**
  Sends the data to the gnuplot and draws the window
  which contains the plot.
  */
void asrVisualizer::sendPlotToGnuplot()
{
    ROS_ASSERT(mGnuplotHandler);
    //Prevent system from trying to send data to non-initialized gnuplot handler
    if(!mGnuplotHandler)
      throw std::runtime_error("Cannot show non-existing gnuplot visualization.");

    Eigen::Vector2f startNext = Eigen::Vector2f(nextView.center.x() - nextView.fov.x() * 0.5, nextView.center.y() - nextView.fov.y() * 0.5);
    Eigen::Vector2f endNext = Eigen::Vector2f(nextView.center.x() + nextView.fov.x() * 0.5, nextView.center.y() + nextView.fov.y() * 0.5);
    Eigen::Vector2f startCurrent = Eigen::Vector2f(lastView.center.x() - lastView.fov.x() * 0.5, lastView.center.y() - lastView.fov.y() * 0.5);
    Eigen::Vector2f endCurrent = Eigen::Vector2f(lastView.center.x() + lastView.fov.x() * 0.5, lastView.center.y() + lastView.fov.y() * 0.5);



    *(mGnuplotHandler) << "set object 1 rect from " << startNext.x() << "," << startNext.y() << " to " << endNext.x() << "," << endNext.y() << " fs transparent solid 0.5 fc rgb \"green\"\n";
    *(mGnuplotHandler) << "set object 2 rect from " << startCurrent.x() << "," << startCurrent.y() << " to " << endCurrent.x() << "," << endCurrent.y() << " fs transparent solid 0.5 fc rgb \"red\"\n";
    *(mGnuplotHandler) << "set style line 1 lc rgb \"#0060ad\" lt 1 lw 2 pt 7 ps 1.5\n";


    //Every data combination has in common it wants to be plotted.
    *(mGnuplotHandler) << "plot ";

    //This should be later called in a loop with type and id as parameters
    *(mGnuplotHandler) << "'-' with points ls 1 title \"Found Objects\"";
    //This should be later called in a loop with type and id as parameters
    *(mGnuplotHandler) << ", '-' with points ls 2 title \"Object Hypotheses\"";
    if(mVisitedViews.size() > 0) *(mGnuplotHandler) << ", '-' with linespoints ls 3 title \"Camera Movement\" ";

    //End of gnuplot instructions for defining how bars are to be plotted
    *(mGnuplotHandler) << "\n";

    //Later I should call .send(mUnfoundBuffer), on the gnuplothandler handler it returns, as often as the number of objects that I search for.

    //Push bar chart data to gnuplot.
    mGnuplotHandler->send(mFoundBuffer).send(mUnfoundBuffer);
    if(mVisitedViews.size() > 0) mGnuplotHandler->send(mVisitedViews);
    mGnuplotHandler->flush();
}

}
