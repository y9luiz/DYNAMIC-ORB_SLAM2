#include "scored_geometric_segmentation.h"
#include "Frame.h"
#include <exception>
#include <stdexcept>

#include <iostream>

using namespace ORB_SLAM2;

using std::logic_error;

ScoredGeometricSegmentation::ScoredGeometricSegmentation(double distanceThreshold, int scoreThreshold)
: m_distanceThreshold(distanceThreshold), m_scoreThreshold(scoreThreshold)
{
    m_minimumScore = -4;
    m_maximumScore = 4;
    std::cout << "ScoredGeometricSegmentation initialized\n";
    std::cout << "distance threshold " << m_distanceThreshold << "\n";
    std::cout << "score threshold " << m_scoreThreshold << "\n";
    std::cout << "maximum score " << m_maximumScore << "\n";
    std::cout << "minimum score " << m_minimumScore << "\n";
}

void ScoredGeometricSegmentation::removeDynamicPointsFromCurrentFrame(ORB_SLAM2::Frame * currentFrame, ORB_SLAM2::Frame * lastFrame)
{
}

vector<int> ScoredGeometricSegmentation::getDynamicPointsIndexes(ORB_SLAM2::Frame * currentFrame, ORB_SLAM2::Frame * lastFrame)
{
    if(!currentFrame || !lastFrame)
    {   
        std::cout << "current or last frame are nullptr!" << "\n";
        return {};
    }

    m_currentFrame = currentFrame;
    m_lastFrame = lastFrame;

    calculateDynamicPointScores();

    vector<int> dynamicPointsIndexes;

    for(int i = 0; i < m_currentNumberOfFeatures; i++)
    {
        if(m_scores[i] < m_scoreThreshold)
        {
           dynamicPointsIndexes.push_back(i);
        }
    }

    cout << "total de pontos \'dinamicos\': " << dynamicPointsIndexes.size() <<"/" << m_currentNumberOfFeatures << endl;

    return dynamicPointsIndexes;
}


void ScoredGeometricSegmentation::calculateDynamicPointScores()
{
    std::cout << "calculating dynamic points scores\n";
    const auto projectedPoints = projectLastPointsIntoCurrentFrame();

    int badPointsCounter = 0;

    for(auto i = 0; i < projectedPoints.size(); i++)
    {
        if(i >= m_scores.size())
        {
            m_scores.push_back(0);
        }

        const auto & projectedPoint = projectedPoints[i];

        const auto isValidPoint = projectedPoint.x >=0 && projectedPoint.y >=0;

        if(!isValidPoint)
        {
            badPointsCounter++;
            continue;
        }

        const auto & pointTo = m_currentFrame->mvKeysUn[i];
        
        const auto distance = euclidianDistance(projectedPoint,pointTo.pt);

        bool hasMoved = distance >= m_distanceThreshold;

        if(hasMoved)
        {
            if(m_scores[i] > m_minimumScore)
            {
                m_scores[i]--;
            }
        }

        if(!hasMoved)
        {
            if(m_scores[i] < m_maximumScore)
            {
                m_scores[i]++;
            }
        }
    }

    cout << "total of bad points " << badPointsCounter << endl;
}

vector<cv::Point2f> ScoredGeometricSegmentation::projectLastPointsIntoCurrentFrame()
{
   m_currentNumberOfFeatures = min(m_currentFrame->N, m_lastFrame->N);

    if(m_currentNumberOfFeatures <= 0)
    {
        return {};
    }

    const auto & currentFramePose = m_currentFrame->mTcw;
    const auto & currentFrameRotation = currentFramePose.rowRange(0,3).colRange(0,3);
    const auto & currentFrameTranslation = currentFramePose.rowRange(0,3).col(3);

    const auto & lastFramePose = m_lastFrame->mTcw;

    if(currentFramePose.empty() || lastFramePose.empty())
    {
        throw logic_error("current or last frames is empty");
    }

    vector<cv::Point2f> projectedPoints;

    int nullMvpMapPointerCounter = 0;

    for(int i = 0; i < m_currentNumberOfFeatures; i++)
    {        

        bool isValidPoint = false;

        if(!m_lastFrame->mvpMapPoints[i] || m_lastFrame->mvbOutlier[i])
        {
            float u = -1;
            float v =  -1;

            projectedPoints.emplace_back(u,v);
            nullMvpMapPointerCounter++;
            continue;
        }


        cv::Mat poseLastFrame = m_lastFrame->mvpMapPoints[i]->GetWorldPos();
        cv::Mat poseLastFrameTransformed = currentFrameRotation*poseLastFrame + currentFrameTranslation; 

        const auto x = poseLastFrameTransformed.at<float>(0);
        const auto y = poseLastFrameTransformed.at<float>(1);
        const float inverseZ = 1.0/poseLastFrameTransformed.at<float>(2);
        
        isValidPoint = inverseZ >= 0;

        float u = isValidPoint? m_currentFrame->fx*x*inverseZ+m_currentFrame->cx: -1;
        float v = isValidPoint? m_currentFrame->fy*y*inverseZ+m_currentFrame->cy: -1;

        if(u<m_currentFrame->mnMinX || u>m_currentFrame->mnMaxX || v<m_currentFrame->mnMinY || v>m_currentFrame->mnMaxY)
        {
            u = -1;
            v = -1;
        }
        
        projectedPoints.emplace_back(u,v);
    }

    cout << "quantidade de mvp map nulos " << nullMvpMapPointerCounter << endl;

    return projectedPoints;
}


double ScoredGeometricSegmentation::euclidianDistance(cv::Point2f pt1, cv::Point2f pt2)
{
  float x1 = pt1.x;
  float y1 = pt1.y;

  float x2 = pt2.x;
  float y2 = pt2.y;

  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}