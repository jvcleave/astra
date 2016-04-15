#include "DepthUtility.h"
#include <AstraUL/AstraUL.h>
#include "TrackingData.h"
#include <cmath>
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

namespace astra { namespace plugins { namespace hand {

    DepthUtility::DepthUtility(float width, float height, DepthUtilitySettings& settings) :
        m_processingWidth(width),
        m_processingHeight(height),
        m_depthSmoothingFactor(settings.depthSmoothingFactor),
        m_velocityThresholdFactor(settings.velocityThresholdFactor),
        m_maxDepthJumpPercent(settings.maxDepthJumpPercent),
        m_erodeSize(settings.erodeSize),
        m_depthAdjustmentFactor(settings.depthAdjustmentFactor),
        m_minDepth(settings.minDepth),
        m_maxDepth(settings.maxDepth)
    {
        PROFILE_FUNC();
        m_rectElement = cv::getStructuringElement(cv::MORPH_RECT,
                                                  cv::Size(m_erodeSize * 2 + 1, m_erodeSize * 2 + 1),
                                                  cv::Point(m_erodeSize, m_erodeSize));

        reset();
    }

    DepthUtility::~DepthUtility()
    {
        PROFILE_FUNC();
    }

    void DepthUtility::reset()
    {
        PROFILE_FUNC();
        m_matDepthFilled = cv::Mat::zeros(m_processingHeight, m_processingWidth, CV_32FC1);
        m_matDepthFilledMask = cv::Mat::zeros(m_processingHeight, m_processingWidth, CV_8UC1);
        m_matDepthPrevious = cv::Mat::zeros(m_processingHeight, m_processingWidth, CV_32FC1);
        m_matDepthAvg = cv::Mat::zeros(m_processingHeight, m_processingWidth, CV_32FC1);
        m_matDepthVel.create(m_processingHeight, m_processingWidth, CV_32FC1);
        m_matDepthVelErode.create(m_processingHeight, m_processingWidth, CV_32FC1);

        for (int i = 0; i < NUM_DEPTH_VEL_CHUNKS; i++)
        {
            m_maxVel[i] = 0;
        }
    }

    void DepthUtility::processDepthToVelocitySignal(DepthFrame& depthFrame,
                                                    cv::Mat& matDepth,
                                                    cv::Mat& matDepthFullSize,
                                                    cv::Mat& matVelocitySignal)
    {
        PROFILE_FUNC();
        int width = depthFrame.resolutionX();
        int height = depthFrame.resolutionY();

        depthFrameToMat(depthFrame, width, height, matDepthFullSize);

        if (width == m_processingWidth && height == m_processingHeight)
        {
            //target size is original size, just use the same data
            matDepth = matDepthFullSize;
        }
        else
        {
            matDepth.create(m_processingHeight, m_processingWidth, CV_32FC1);
            //convert to the target processing size with nearest neighbor
            cv::resize(matDepthFullSize, matDepth, matDepth.size(), 0, 0, CV_INTER_NN);
        }

        matVelocitySignal = cv::Mat::zeros(m_processingHeight, m_processingWidth, CV_8UC1);

        //fill 0 depth pixels with the value from the previous frame
        fillZeroValues(matDepth, m_matDepthFilled, m_matDepthFilledMask, m_matDepthPrevious);

        //accumulate current frame to average using smoothing factor

        cv::accumulateWeighted(m_matDepthFilled, m_matDepthAvg, m_depthSmoothingFactor);

        filterZeroValuesAndJumps(m_matDepthFilled,
                                 m_matDepthPrevious,
                                 m_matDepthAvg,
                                 m_matDepthFilledMask,
                                 m_maxDepthJumpPercent);

        //current minus average, scaled by average = velocity as a percent change

        m_matDepthVel = (m_matDepthFilled - m_matDepthAvg) / m_matDepthAvg;

        adjust_velocities_for_depth(matDepth, m_matDepthVel);

        //erode to eliminate single pixel velocity artifacts
        m_matDepthVelErode = cv::abs(m_matDepthVel);
        cv::erode(m_matDepthVelErode, m_matDepthVelErode, m_rectElement);
        //cv::dilate(m_matDepthVelErode, m_matDepthVelErode, m_rectElement);

        thresholdVelocitySignal(m_matDepthVelErode,
                                matVelocitySignal,
                                m_velocityThresholdFactor);

        //analyze_velocities(matDepth, m_matDepthVelErode);
    }

    void DepthUtility::depthFrameToMat(DepthFrame& depthFrameSrc,
                                       const int width,
                                       const int height,
                                       cv::Mat& matTarget)
    {
        PROFILE_FUNC();
        //ensure initialized
        matTarget.create(height, width, CV_32FC1);

        const int16_t* depthData = depthFrameSrc.data();

        for (int y = 0; y < height; ++y)
        {
            float* row = matTarget.ptr<float>(y);
            for (int x = 0; x < width; ++x)
            {
                float depth = static_cast<float>(*depthData);
                *row = depth;
                ++row;
                ++depthData;
            }
        }
    }

    void DepthUtility::fillZeroValues(cv::Mat& matDepth,
                                      cv::Mat& matDepthFilled,
                                      cv::Mat& matDepthFilledMask,
                                      cv::Mat& matDepthPrevious)
    {
        PROFILE_FUNC();
        int width = matDepth.cols;
        int height = matDepth.rows;

        for (int y = 0; y < height; ++y)
        {
            float* depthRow = matDepth.ptr<float>(y);
            float* prevDepthRow = matDepthPrevious.ptr<float>(y);
            float* filledDepthRow = matDepthFilled.ptr<float>(y);
            uint8_t* filledDepthMaskRow = matDepthFilledMask.ptr<uint8_t>(y);

            for (int x = 0; x < width; ++x)
            {
                float depth = *depthRow;

                if (depth == 0)
                {
                    depth = *prevDepthRow;
                    *filledDepthMaskRow = FillMaskType::Filled;
                }
                else
                {
                    *filledDepthMaskRow = FillMaskType::Normal;
                }

                *filledDepthRow = depth;

                ++depthRow;
                ++prevDepthRow;
                ++filledDepthRow;
                ++filledDepthMaskRow;
            }
        }
    }

    void DepthUtility::filterZeroValuesAndJumps(cv::Mat& matDepth,
                                                cv::Mat& matDepthPrevious,
                                                cv::Mat& matDepthAvg,
                                                cv::Mat& matDepthFilledMask,
                                                const float maxDepthJumpPercent)
    {
        PROFILE_FUNC();
        int width = matDepth.cols;
        int height = matDepth.rows;

        for (int y = 0; y < height; ++y)
        {
            float* depthRow = matDepth.ptr<float>(y);
            float* prevDepthRow = matDepthPrevious.ptr<float>(y);
            float* avgRow = matDepthAvg.ptr<float>(y);
            uint8_t* filledDepthMaskRow = matDepthFilledMask.ptr<uint8_t>(y);

            for (int x = 0; x < width; ++x, ++depthRow, ++prevDepthRow, ++filledDepthMaskRow)
            {
                float depth = *depthRow;
                float previousDepth = *prevDepthRow;
                uint8_t fillType = *filledDepthMaskRow;

                //calculate percent change since last frame
                float deltaPercent = (depth - previousDepth) / previousDepth;
                float absDeltaPercent = std::fabs(deltaPercent);
                bool movingAway = deltaPercent > 0;

                //suppress signal if either current or previous pixel are invalid
                bool isZeroDepth = (0 == depth || 0 == previousDepth);

                //suppress signal when a pixel was artificially filled
                bool isFilled = (fillType == FillMaskType::Filled);

                //suppress signal when a pixel jumps a long distance from near to far
                bool isJumpingAway = (absDeltaPercent > maxDepthJumpPercent && movingAway);

                if (isZeroDepth || isFilled || isJumpingAway)
                {
                    //set the average to the current depth, and set velocity to zero
                    //this suppresses the velocity signal for edge jumping artifacts
                    avgRow[x] = depth;
                }

                *prevDepthRow = depth;
            }
        }
    }

    void DepthUtility::thresholdVelocitySignal(cv::Mat& matVelocityFiltered,
                                               cv::Mat& matVelocitySignal,
                                               const float velocityThresholdFactor)
    {
        PROFILE_FUNC();
        int width = matVelocitySignal.cols;
        int height = matVelocitySignal.rows;

        for (int y = 0; y < height; ++y)
        {
            float* velFilteredRow = matVelocityFiltered.ptr<float>(y);
            uint8_t* velocitySignalRow = matVelocitySignal.ptr<uint8_t>(y);

            for (int x = 0; x < width; ++x, ++velFilteredRow, ++velocitySignalRow)
            {
                //matVelocityFiltered is already abs(vel)
                float velFiltered = *velFilteredRow;

                if (velFiltered > velocityThresholdFactor)
                {
                    *velocitySignalRow = PixelType::Foreground;
                }
                else
                {
                    *velocitySignalRow = PixelType::Background;
                }
            }
        }
    }

    void DepthUtility::adjust_velocities_for_depth(cv::Mat& matDepth, cv::Mat& matVelocityFiltered)
    {
        PROFILE_FUNC();
        if (m_depthAdjustmentFactor == 0)
        {
            return;
        }

        int width = matDepth.cols;
        int height = matDepth.rows;

        for (int y = 0; y < height; ++y)
        {
            float* depthRow = matDepth.ptr<float>(y);
            float* velFilteredRow = matVelocityFiltered.ptr<float>(y);

            for (int x = 0; x < width; ++x, ++depthRow, ++velFilteredRow)
            {
                float depth = *depthRow;
                if (depth != 0.0f)
                {
                    float& velFiltered = *velFilteredRow;
                    if (depth > m_minDepth && depth < m_maxDepth)
                    {
                        float depthM = depth / 1000.0f;
                        velFiltered /= depthM * m_depthAdjustmentFactor;
                    }
                    else
                    {
                        velFiltered = 0;
                    }
                }
            }
        }
    }

    int DepthUtility::depth_to_chunk_index(float depth)
    {
        PROFILE_FUNC();
        if (depth == 0 || depth < MIN_CHUNK_DEPTH || depth > MAX_CHUNK_DEPTH)
        {
            return -1;
        }
        const float chunkRange = MAX_CHUNK_DEPTH - MIN_CHUNK_DEPTH;
        const float normDepth = (depth - MIN_CHUNK_DEPTH) / chunkRange;

        return std::min(NUM_DEPTH_VEL_CHUNKS - 1, static_cast<int>(NUM_DEPTH_VEL_CHUNKS * normDepth));
    }

    void DepthUtility::analyze_velocities(cv::Mat& matDepth, cv::Mat& matVelocityFiltered)
    {
        PROFILE_FUNC();
        int width = matDepth.cols;
        int height = matDepth.rows;

        for (int i = 0; i < NUM_DEPTH_VEL_CHUNKS; i++)
        {
            m_maxVel[i] *= m_velErodeFactor;
            m_depthCount[i] = 0;
        }

        for (int y = 0; y < height; ++y)
        {
            float* depthRow = matDepth.ptr<float>(y);
            float* velFilteredRow = matVelocityFiltered.ptr<float>(y);

            for (int x = 0; x < width; ++x, ++depthRow, ++velFilteredRow)
            {
                //matVelocityFiltered is already abs(vel)

                float depth = *depthRow;
                int chunkIndex = depth_to_chunk_index(depth);
                if (chunkIndex >= 0 && depth != 0)
                {
                    ++m_depthCount[chunkIndex];

                    float& maxVel = m_maxVel[chunkIndex];
                    float velFiltered = *velFilteredRow;

                    if (velFiltered > maxVel)
                    {
                        maxVel = velFiltered;
                    }
                }
            }
        }

        const float chunkRange = MAX_CHUNK_DEPTH - MIN_CHUNK_DEPTH;
        const float chunk_size = chunkRange / NUM_DEPTH_VEL_CHUNKS;

        for (int i = 0; i < NUM_DEPTH_VEL_CHUNKS; i++)
        {
            float maxVel = m_maxVel[i];
            int count = m_depthCount[i];
            float startDepth = (MIN_CHUNK_DEPTH + i * chunk_size) / 1000.0f;
            float endDepth = (MIN_CHUNK_DEPTH + (1 + i) * chunk_size) / 1000.0f;
            float ratio = 0;
            if (endDepth != 0.0f)
            {
                ratio = maxVel / endDepth;
            }

            printf("[%.1fm %d,%f,%f] ", startDepth, count, maxVel, ratio);
            if (i % 3 == 2)
            {
                printf("\n");
            }
        }
        printf("[%.1fm]\n\n", static_cast<int>(MAX_CHUNK_DEPTH)/1000.0f);
    }
}}}
