#pragma once

#include "Frame.h"
#include <opencv2/core/types.hpp>

class ScoredGeometricSegmentation 
{
    public:
        ScoredGeometricSegmentation( double distanceThreshold = 40, int scoreThreshold = -2);

        void removeDynamicPointsFromCurrentFrame(ORB_SLAM2::Frame * currentFrame, ORB_SLAM2::Frame * lastFrame);

        vector<int> getDynamicPointsIndexes(ORB_SLAM2::Frame * currentFrame, ORB_SLAM2::Frame * lastFrame);

    private:
        
        std::vector<cv::Point2f> projectLastPointsIntoCurrentFrame();

        void calculateDynamicPointScores();

        double euclidianDistance(cv::Point2f pt1, cv::Point2f pt2);
        

        ORB_SLAM2::Frame * m_currentFrame;
        ORB_SLAM2::Frame * m_lastFrame;

        std::vector<int> m_scores;
        double m_distanceThreshold;
        int m_scoreThreshold;
        int m_minimumScore;
        int m_maximumScore;
        int m_currentNumberOfFeatures;
};