#pragma once
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

namespace ASR
{
	class simpleGrid
	{
	private:
		int grid[128][128];
		float gridWidth;
		float gridHeight;
		int correctX;
		int correctY;
        Eigen::Vector2f rangeMin;
        Eigen::Vector2f rangeMax;
        Eigen::Vector2f fieldOfView;
		
	public:
		simpleGrid(Eigen::Vector2f rangeMin, Eigen::Vector2f rangeMax, Eigen::Vector2f fieldOfView)
		{
			for (int i=0;i<128;i++)
			{
				for (int j=0;j<128;j++)
				{
					grid[i][j] = 0;
				}
			}
            this->rangeMax = rangeMax;
            this->rangeMin = rangeMin;
            this->fieldOfView = fieldOfView;

            gridWidth = 120 / fieldOfView.x();
            gridHeight = 120 / fieldOfView.y();
			
            correctX = abs(rangeMin.x()) / gridWidth;
            correctY = abs(rangeMin.y()) / gridHeight;
		}

        void addPoint(float x, float y)
        {
            addPoint(Eigen::Vector2f(x,y));
        }
		
		void addPoint(Eigen::Vector2f point)
		{
			if (point.x() < rangeMin.x() 
				|| point.x() > rangeMax.x() 
				|| point.y() < rangeMin.y()
				|| point.y() > rangeMax.y())
				{
					// point out of range
					return;
				}
				
				int i = int(point.x() / gridWidth) + correctX;
				int j = int(point.y() / gridHeight) + correctY;
				
				grid[i][j]++;
		}
		
		Eigen::Vector2f getCenterOfNextView()
		{
			int max = 0;
			int iResult = 0;
			int jResult = 0;
			
			for (int i=0;i<128;i++)
			{
				for (int j=0;j<128;j++)
				{
					if(grid[i][j] > max) 
					{
						max = grid[i][j];
						iResult = i;
						jResult = j;

                        // resetGruid
                        grid[i][j] = 0;
					}
				}
			}
            iResult -= correctX;
            jResult -= correctY;
		
            return Eigen::Vector2f( iResult * gridWidth + (fieldOfView.x() * 0.5f),
                                    jResult * gridHeight + (fieldOfView.y() * 0.5f));
		}
    };

    typedef boost::shared_ptr<simpleGrid> simpleGridPtr;
}
