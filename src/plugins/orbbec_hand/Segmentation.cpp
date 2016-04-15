#include "TrackingData.h"
#include <queue>
#include "ScalingCoordinateMapper.h"
#include <cmath>
#include "Segmentation.h"
#include "constants.h"
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 
#define PROFILE_BLOCK(...) 
#include <Astra/Plugins/PluginLogger.h>

#define MAX_DEPTH 10000

namespace astra { namespace plugins { namespace hand { namespace segmentation {

    struct PointTTL
    {
        int x;
        int y;
        float ttl;

        PointTTL(int x, int y, float ttl) :
            x(x),
            y(y),
            ttl(ttl)
        { }
    };

    static void enqueue_neighbors(cv::Mat& matVisited,
                                 std::queue<PointTTL>& pointQueue,
                                 const PointTTL& pt)
    {
        PROFILE_FUNC();
        const int& x = pt.x;
        const int& y = pt.y;
        const int width = matVisited.cols;
        const int height = matVisited.rows;

        if (x < 1 || x > width - 2 ||
            y < 1 || y > height - 2)
        {
            return;
        }

        const float& ttlRef = pt.ttl;

        char& rightVisited = matVisited.at<char>(y, x+1);
        if (0 == rightVisited)
        {
            rightVisited = 1;
            pointQueue.push(PointTTL(x+1, y, ttlRef));
        }

        char& leftVisited = matVisited.at<char>(y, x-1);
        if (0 == leftVisited)
        {
            leftVisited = 1;
            pointQueue.push(PointTTL(x-1, y, ttlRef));
        }

        char& downVisited = matVisited.at<char>(y+1, x);
        if (0 == downVisited)
        {
            downVisited = 1;
            pointQueue.push(PointTTL(x, y+1, ttlRef));
        }

        char& upVisited = matVisited.at<char>(y-1, x);
        if (0 == upVisited)
        {
            upVisited = 1;
            pointQueue.push(PointTTL(x, y-1, ttlRef));
        }
    }

    static cv::Point find_nearest_in_range_pixel(TrackingData& data,
                                                cv::Mat& matVisited)
    {
        PROFILE_FUNC();
        assert(matVisited.size() == data.matrices.depth.size());
        const float referenceAreaSqrt = data.referenceAreaSqrt;
        if (referenceAreaSqrt == 0)
        {
            return INVALID_POINT;
        }

        const float minDepth = data.referenceWorldPosition.z - data.settings.segmentationBandwidthDepthNear;
        const float maxDepth = data.referenceWorldPosition.z + data.settings.segmentationBandwidthDepthFar;
        const float maxSegmentationDist = data.settings.maxSegmentationDist;
        cv::Mat& depthMatrix = data.matrices.depth;
        cv::Mat& searchedMatrix = data.matrices.foregroundSearched;

        std::queue<PointTTL> pointQueue;

        pointQueue.push(PointTTL(data.seedPosition.x,
                                 data.seedPosition.y,
                                 maxSegmentationDist));

        matVisited.at<char>(data.seedPosition) = 1;

        while (!pointQueue.empty())
        {
            PointTTL pt = pointQueue.front();
            pointQueue.pop();
            const int& x = pt.x;
            const int& y = pt.y;
            float& ttlRef = pt.ttl;

            if (ttlRef <= 0)
            {
                continue;
            }

            searchedMatrix.at<char>(y, x) = PixelType::SearchedFromOutOfRange;

            float depth = depthMatrix.at<float>(y, x);
            bool pointInRange = depth != 0 && depth > minDepth && depth < maxDepth;

            if (pointInRange)
            {
                return cv::Point(x, y);
            }

            ttlRef -= referenceAreaSqrt;

            enqueue_neighbors(matVisited, pointQueue, pt);
        }

        return INVALID_POINT;
    }

    static float segment_foreground_and_get_average_depth(TrackingData& data)
    {
        PROFILE_FUNC();
        const float& maxSegmentationDist = data.settings.maxSegmentationDist;
        const SegmentationVelocityPolicy& velocitySignalPolicy = data.velocityPolicy;
        const float seedDepth = data.matrices.depth.at<float>(data.seedPosition);
        const float referenceAreaSqrt = data.referenceAreaSqrt;
        cv::Mat& depthMatrix = data.matrices.depth;
        cv::Mat& velocitySignalMatrix = data.matrices.velocitySignal;
        cv::Mat& segmentationMatrix = data.matrices.layerSegmentation;
        cv::Mat& searchedMatrix = data.matrices.foregroundSearched;

        std::queue<PointTTL> pointQueue;

        double totalDepth = 0;
        int depthCount = 0;

        const float bandwidthDepth = data.settings.segmentationBandwidthDepthNear;

        //does the seed point start in range?
        //If not, it will search outward until it finds in range pixels
        const float minDepth = data.referenceWorldPosition.z - bandwidthDepth;
        const float maxDepth = data.referenceWorldPosition.z + data.settings.segmentationBandwidthDepthFar;

        cv::Mat matVisited = cv::Mat::zeros(depthMatrix.size(), CV_8UC1);

        cv::Point seedPosition = data.seedPosition;

        bool seedInRange = seedDepth != 0 && seedDepth > minDepth && seedDepth < maxDepth;
        if (!seedInRange)
        {
            seedPosition = find_nearest_in_range_pixel(data, matVisited);
            if (seedPosition == INVALID_POINT)
            {
                //No in range pixels found, no foreground to set
                return 0.0f;
            }
        }

        pointQueue.push(PointTTL(seedPosition.x,
                                 seedPosition.y,
                                 maxSegmentationDist));

        matVisited.at<char>(seedPosition) = 1;

        while (!pointQueue.empty())
        {
            PointTTL pt = pointQueue.front();
            pointQueue.pop();
            const int& x = pt.x;
            const int& y = pt.y;
            float& ttlRef = pt.ttl;

            if (velocitySignalPolicy == VELOCITY_POLICY_RESET_TTL &&
                velocitySignalMatrix.at<char>(y, x) == PixelType::Foreground)
            {
                ttlRef = maxSegmentationDist;
            }

            float depth = depthMatrix.at<float>(y, x);
            bool pointOutOfRange = depth == 0 ||
                                   depth < minDepth ||
                                   depth > maxDepth;

            if (ttlRef <= 0)
            {
                segmentationMatrix.at<char>(y, x) = PixelType::ForegroundOutOfRangeEdge;
                continue;
            }
            else if (pointOutOfRange)
            {
                segmentationMatrix.at<char>(y, x) = PixelType::ForegroundNaturalEdge;
                continue;
            }

            totalDepth += depth;
            ++depthCount;

            searchedMatrix.at<char>(y, x) = PixelType::Searched;
            segmentationMatrix.at<char>(y, x) = PixelType::Foreground;

            ttlRef -= referenceAreaSqrt;

            enqueue_neighbors(matVisited, pointQueue, pt);
        }

        if (depthCount > 0)
        {
            float averageDepth = static_cast<float>(totalDepth / depthCount);
            return averageDepth;
        }
        else
        {
            return 0.0f;
        }
    }

    void calculate_layer_score(TrackingData& data, const float layerAverageDepth)
    {
        PROFILE_FUNC();
        cv::Mat& edgeDistanceMatrix = data.matrices.layerEdgeDistance;
        const float depthFactor = data.settings.depthScoreFactor;
        const float heightFactor = data.settings.heightScoreFactor;
        const float edgeDistanceFactor = data.settings.edgeDistanceScoreFactor;
        const float targetEdgeDist = data.settings.targetEdgeDistance;
        cv::Mat& layerScoreMatrix = data.matrices.layerScore;
        const float pointInertiaFactor = data.settings.pointInertiaFactor;
        const float pointInertiaRadius = data.settings.pointInertiaRadius;
        const astra::Vector3f* worldPoints = data.matrices.worldPoints;

        layerScoreMatrix = cv::Mat::zeros(data.matrices.depth.size(), CV_32FC1);

        ScalingCoordinateMapper mapper = get_scaling_mapper(data.matrices);

        auto seedWorldPosition = astra::Vector3f(data.referenceWorldPosition.x,
                                                    data.referenceWorldPosition.y,
                                                    data.referenceWorldPosition.z);

        int width = data.matrices.depth.cols;
        int height = data.matrices.depth.rows;

        int edgeRadius = mapper.scale() * width / 32;
        int minX = edgeRadius - 1;
        int maxX = width - edgeRadius;
        int minY = edgeRadius - 1;
        int maxY = height - edgeRadius;

        for (int y = 0; y < height; y++)
        {
            float* edgeDistanceRow = edgeDistanceMatrix.ptr<float>(y);
            float* layerScoreRow = layerScoreMatrix.ptr<float>(y);

            for (int x = 0; x < width; ++x,
                                       ++worldPoints,
                                       ++edgeDistanceRow,
                                       ++layerScoreRow)
            {
                astra::Vector3f worldPosition = *worldPoints;
                if (worldPosition.z != 0 && x > minX && x < maxX && y > minY && y < maxY)
                {
                    //start with arbitrary large value to prevent scores from going negative
                    //(which is ok algorithmically, but debug visuals don't like it)
                    float score = 10000;

                    score += worldPosition.y * heightFactor;

                    float depthDiff = 1.0 - worldPosition.z / layerAverageDepth;
                    score += depthDiff * depthFactor;

                    if (pointInertiaRadius > 0)
                    {
                        auto vector = worldPosition - seedWorldPosition;
                        float length = vector.length();
                        float distFromSeedNorm = std::max(0.0f, std::min(1.0f,
                                                    length / pointInertiaRadius));
                        score += (1.0f - distFromSeedNorm) * pointInertiaFactor;
                    }

                    float edgeDistance = *edgeDistanceRow;
                    float edgeScore = edgeDistanceFactor * std::min(edgeDistance, targetEdgeDist) / targetEdgeDist;

                    score += edgeScore;

                    *layerScoreRow = score;
                }
                else
                {
                    *layerScoreRow = 0;
                }
            }
        }
    }


    bool test_point_in_range(TrackingMatrices& matrices,
                             const cv::Point& targetPoint,
                             TestBehavior outputLog)
    {
        PROFILE_FUNC();
        if (targetPoint == segmentation::INVALID_POINT ||
            targetPoint.x < 0 || targetPoint.x >= matrices.depth.cols ||
            targetPoint.y < 0 || targetPoint.y >= matrices.depth.rows)
        {
            if (outputLog == TEST_BEHAVIOR_LOG)
            {
                LOG_INFO("PointProcessor", "test_point_in_range failed: position: (%d, %d)",
                              targetPoint.x,
                              targetPoint.y);
            }
            return false;
        }

        if (outputLog == TEST_BEHAVIOR_LOG)
        {
            LOG_INFO("PointProcessor", "test_point_in_range success: position: (%d, %d)",
                          targetPoint.x,
                          targetPoint.y);
        }

        return true;
    }

    float get_point_area(TrackingMatrices& matrices,
                         AreaTestSettings& settings,
                         const cv::Point& point)
    {
        PROFILE_FUNC();
        auto scalingMapper = get_scaling_mapper(matrices);

        float area = count_neighborhood_area(matrices.layerSegmentation,
                                             matrices.depth,
                                             matrices.area,
                                             point,
                                             settings.areaBandwidth,
                                             settings.areaBandwidthDepth,
                                             scalingMapper);

        return area;
    }


    float get_point_area_integral(TrackingMatrices& matrices,
                                  cv::Mat& integralArea,
                                  AreaTestSettings& settings,
                                  const cv::Point& point)
    {
        //PROFILE_FUNC();
        auto scalingMapper = get_scaling_mapper(matrices);

        float area = count_neighborhood_area_integral(matrices.depth,
                                                      integralArea,
                                                      point,
                                                      settings.areaBandwidth,
                                                      scalingMapper);

        return area;
    }

    bool test_point_area_core(float area,
                              AreaTestSettings& settings,
                              TestPhase phase,
                              TestBehavior outputLog)
    {
        //PROFILE_FUNC();
        float minArea = settings.minArea;
        const float maxArea = settings.maxArea;
        if (phase == TEST_PHASE_UPDATE)
        {
            //minimum of 0 during update phase
            minArea = 0;
        }

        bool validPointArea = area > minArea && area < maxArea;

        if (outputLog == TEST_BEHAVIOR_LOG)
        {
            if (validPointArea)
            {
                LOG_INFO("Segmentation", "test_point_area passed: area %f within [%f, %f]",
                              area,
                              minArea,
                              maxArea);
            }
            else
            {
                LOG_INFO("Segmentation", "test_point_area failed: area %f not within [%f, %f]",
                              area,
                              minArea,
                              maxArea);
            }
        }

        return validPointArea;
    }

    bool test_point_area(TrackingMatrices& matrices,
                         AreaTestSettings& settings,
                         const cv::Point& targetPoint,
                         TestPhase phase,
                         TestBehavior outputLog)
    {
        PROFILE_FUNC();
        float area = get_point_area(matrices, settings, targetPoint);

        return test_point_area_core(area, settings, phase, outputLog);
    }


    bool test_point_area_integral(TrackingMatrices& matrices,
                                  cv::Mat& integralArea,
                                  AreaTestSettings& settings,
                                  const cv::Point& targetPoint,
                                  TestPhase phase,
                                  TestBehavior outputLog)
    {
        PROFILE_FUNC();
        float area = get_point_area_integral(matrices, integralArea, settings, targetPoint);

        return test_point_area_core(area, settings, phase, outputLog);
    }

    float get_percent_natural_edges(cv::Mat& matDepth,
                                    cv::Mat& matSegmentation,
                                    const cv::Point& center,
                                    const float bandwidth,
                                    const ScalingCoordinateMapper& mapper)
    {
        PROFILE_FUNC();
        int width = matDepth.cols;
        int height = matDepth.rows;
        if (center.x < 0 || center.y < 0 ||
            center.x >= width || center.y >= height)
        {
            return 0;
        }

        float startingDepth = matDepth.at<float>(center);

        cv::Point topLeft = mapper.offset_pixel_location_by_mm(center, -bandwidth, bandwidth, startingDepth);

        int offsetX = center.x - topLeft.x;
        int offsetY = center.y - topLeft.y;
        cv::Point bottomRight(center.x + offsetX, center.y + offsetY);

        int32_t x0 = MAX(0, topLeft.x);
        int32_t y0 = MAX(0, topLeft.y);
        int32_t x1 = MIN(width - 1, bottomRight.x);
        int32_t y1 = MIN(height - 1, bottomRight.y);

        int naturalEdgeCount = 0;
        int totalEdgeCount = 0;

        for (int y = y0; y <= y1; y++)
        {
            char* segmentationRow = matSegmentation.ptr<char>(y);

            segmentationRow += x0;
            for (int x = x0; x <= x1; ++x, ++segmentationRow)
            {
                char segmentation = *segmentationRow;
                if (segmentation == PixelType::ForegroundNaturalEdge)
                {
                    ++naturalEdgeCount;
                    ++totalEdgeCount;
                }
                else if (segmentation == PixelType::ForegroundOutOfRangeEdge)
                {
                    ++totalEdgeCount;
                }
            }
        }

        float percentNaturalEdges = 0;
        if (totalEdgeCount > 0)
        {
            percentNaturalEdges = naturalEdgeCount / static_cast<float>(totalEdgeCount);
        }

        return percentNaturalEdges;
    }

    bool test_natural_edges(TrackingMatrices& matrices,
                            NaturalEdgeTestSettings& settings,
                            const cv::Point& targetPoint,
                            TestPhase phase,
                            TestBehavior outputLog)
    {
        PROFILE_FUNC();

        auto scalingMapper = get_scaling_mapper(matrices);
        float percentNaturalEdges = get_percent_natural_edges(matrices.depth,
                                                              matrices.layerSegmentation,
                                                              targetPoint,
                                                              settings.naturalEdgeBandwidth,
                                                              scalingMapper);

        float minPercentNaturalEdges = settings.minPercentNaturalEdges;

        bool passed = percentNaturalEdges >= minPercentNaturalEdges;

        if (outputLog == TEST_BEHAVIOR_LOG)
        {
            if (passed)
            {
                LOG_INFO("Segmentation", "test_natural_edges passed: %f (minimum %f)",
                              percentNaturalEdges,
                              minPercentNaturalEdges);
            }
            else
            {
                LOG_INFO("Segmentation", "test_natural_edges failed: %f (minimum %f)",
                              percentNaturalEdges,
                              minPercentNaturalEdges);
            }
        }
        return passed;
    }

    bool test_foreground_radius_percentage(TrackingMatrices& matrices,
                                           CircumferenceTestSettings& settings,
                                           const cv::Point& targetPoint,
                                           TestPhase phase,
                                           TestBehavior outputLog)
    {
        PROFILE_FUNC();
        auto scalingMapper = get_scaling_mapper(matrices);

        std::vector<astra::Vector2i>& points = matrices.layerCirclePoints;

        float percentForeground1 = get_max_sequential_circumference_percentage(matrices.depth,
                                                                               matrices.layerSegmentation,
                                                                               targetPoint,
                                                                               settings.foregroundRadius1,
                                                                               scalingMapper,
                                                                               points);

        float percentForeground2 = get_max_sequential_circumference_percentage(matrices.depth,
                                                                               matrices.layerSegmentation,
                                                                               targetPoint,
                                                                               settings.foregroundRadius2,
                                                                               scalingMapper,
                                                                               points);

        float minPercent1 = settings.foregroundRadiusMinPercent1;
        float minPercent2 = settings.foregroundRadiusMinPercent2;
        const float maxPercent1 = settings.foregroundRadiusMaxPercent1;
        const float maxPercent2 = settings.foregroundRadiusMaxPercent2;

        if (phase == TEST_PHASE_UPDATE)
        {
            //no minimum during update phase
            minPercent1 = 0;
            minPercent2 = 0;
        }

        bool passTest1 = percentForeground1 >= minPercent1 &&
                         percentForeground1 <= maxPercent1;

        bool passTest2 = percentForeground2 >= minPercent2 &&
                         percentForeground2 <= maxPercent2;

        bool passed = passTest1 && passTest2;

        if (outputLog == TEST_BEHAVIOR_LOG)
        {
            if (passed)
            {
                LOG_INFO("Segmentation", "test_foreground_radius_percentage passed: perc1 %f [%f,%f] perc2 %f [%f,%f]",
                              percentForeground1,
                              minPercent1,
                              maxPercent1,
                              percentForeground2,
                              minPercent2,
                              maxPercent2);
            }
            else
            {
                LOG_INFO("Segmentation", "test_foreground_radius_percentage failed: perc1 %f [%f,%f] perc2 %f [%f,%f]",
                              percentForeground1,
                              minPercent1,
                              maxPercent1,
                              percentForeground2,
                              minPercent2,
                              maxPercent2);
            }
        }
        return passed;
    }

    cv::Mat& calculate_integral_area(TrackingMatrices& matrices)
    {
        PROFILE_FUNC();
        cv::Mat& segmentationMatrix = matrices.layerSegmentation;
        cv::Mat& areaMatrix = matrices.area;
        cv::Mat& integralAreaMatrix = matrices.layerIntegralArea;
        integralAreaMatrix = cv::Mat::zeros(matrices.depth.size(), CV_32FC1);

        int width = matrices.depth.cols;
        int height = matrices.depth.rows;

        float* lastIntegralAreaRow = nullptr;
        for (int y = 0; y < height; y++)
        {
            char* segmentationRow = segmentationMatrix.ptr<char>(y);
            float* areaRow = areaMatrix.ptr<float>(y);
            float* integralAreaRow = integralAreaMatrix.ptr<float>(y);
            float* integralAreaRowStart = integralAreaRow;

            float leftArea = 0;
            float upLeftArea = 0;

            for (int x = 0; x < width; ++x,
                                       ++areaRow,
                                       ++integralAreaRow,
                                       ++segmentationRow)
            {
                float upArea = 0;
                if (lastIntegralAreaRow != nullptr)
                {
                    upArea = *lastIntegralAreaRow;
                    ++lastIntegralAreaRow;
                }

                float currentArea = 0;

                if (*segmentationRow == PixelType::Foreground)
                {
                    currentArea = *areaRow;
                }

                float area = currentArea + leftArea + upArea - upLeftArea;

                *integralAreaRow = area;

                leftArea = area;
                upLeftArea = upArea;
            }

            lastIntegralAreaRow = integralAreaRowStart;
        }

        return integralAreaMatrix;
    }

    bool test_single_point(TrackingData& data, cv::Point seedPosition)
    {
        auto matrices = data.matrices;

        auto areaTestSettings = data.settings.areaTestSettings;
        auto circumferenceTestSettings = data.settings.circumferenceTestSettings;
        auto naturalEdgeTestSettings = data.settings.naturalEdgeTestSettings;
        auto integralArea = matrices.layerIntegralArea;

        TestPhase phase = data.phase;
        TestBehavior outputTestLog = TEST_BEHAVIOR_NONE;

        bool validPointArea = test_point_area_integral(matrices,
                                                       integralArea,
                                                       areaTestSettings,
                                                       seedPosition,
                                                       phase,
                                                       outputTestLog);
        bool validRadiusTest = false;

        if (validPointArea)
        {
            validRadiusTest = test_foreground_radius_percentage(matrices,
                                                                circumferenceTestSettings,
                                                                seedPosition,
                                                                phase,
                                                                outputTestLog);
        }

        bool validNaturalEdgeTest = false;
        if (validRadiusTest)
        {
            validNaturalEdgeTest = test_natural_edges(matrices,
                                                      naturalEdgeTestSettings,
                                                      seedPosition,
                                                      phase,
                                                      outputTestLog);
        }

        bool passesAllTests = (validPointArea && validRadiusTest && validNaturalEdgeTest);

        return passesAllTests;
    }

    ForegroundStatus create_test_pass_from_foreground(TrackingData& data)
    {
        PROFILE_FUNC();
        auto matrices = data.matrices;
        cv::Mat& segmentationMatrix = matrices.layerSegmentation;
        cv::Mat& testPassMatrix = matrices.layerTestPassMap;

        testPassMatrix = cv::Mat::zeros(segmentationMatrix.size(), CV_8UC1);

        int width = matrices.depth.cols;
        int height = matrices.depth.rows;

        auto areaTestSettings = data.settings.areaTestSettings;
        auto circumferenceTestSettings = data.settings.circumferenceTestSettings;
        auto naturalEdgeTestSettings = data.settings.naturalEdgeTestSettings;

        auto integralArea = matrices.layerIntegralArea;

        TestPhase phase = data.phase;
        TestBehavior outputTestLog = TEST_BEHAVIOR_NONE;

        ForegroundStatus status = FOREGROUND_EMPTY;

        int xskip = 1;
        int yskip = 1;
        bool downscale = false;
        if (data.phase == TEST_PHASE_CREATE ||
            data.referenceWorldPosition.z < data.settings.maxDepthToDownscaleTestPass)
        {
            //during create cycle or if our point is close enough,
            //downscale test pass map by 2 to save computation
            downscale = true;
            xskip = 2;
            yskip = 2;
        }
        const int maxY = height - yskip;

        for (int y = 0; y <= maxY; y += yskip)
        {
            char* segmentationRow = segmentationMatrix.ptr<char>(y);
            char* testPassRow = testPassMatrix.ptr<char>(y);
            char* testPassRowNext = testPassMatrix.ptr<char>(y+1);

            for (int x = 0; x < width; x += xskip,
                                       segmentationRow += xskip,
                                       testPassRow += xskip,
                                       testPassRowNext += xskip)
            {
                if (*segmentationRow != PixelType::Foreground)
                {
                    continue;
                }

                cv::Point seedPosition(x, y);
                bool validPointArea = test_point_area_integral(matrices,
                                                               integralArea,
                                                               areaTestSettings,
                                                               seedPosition,
                                                               phase,
                                                               outputTestLog);
                bool validRadiusTest = false;

                if (validPointArea)
                {
                    validRadiusTest = test_foreground_radius_percentage(matrices,
                                                                        circumferenceTestSettings,
                                                                        seedPosition,
                                                                        phase,
                                                                        outputTestLog);
                }

                bool validNaturalEdgeTest = false;
                if (validRadiusTest)
                {
                    validNaturalEdgeTest = test_natural_edges(matrices,
                                                              naturalEdgeTestSettings,
                                                              seedPosition,
                                                              phase,
                                                              outputTestLog);
                }

                if (validPointArea && validRadiusTest && validNaturalEdgeTest)
                {
                    status = FOREGROUND_HAS_POINTS;
                    *testPassRow = PixelType::Foreground;
                    if (downscale)
                    {
                        *(testPassRow+1) = PixelType::Foreground;
                        *testPassRowNext = PixelType::Foreground;
                        *(testPassRowNext+1) = PixelType::Foreground;
                    }
                }
            }
        }
        return status;
    }

    cv::Point track_point_from_seed(TrackingData& data)
    {
        PROFILE_FUNC();
        cv::Size size = data.matrices.depth.size();
        data.matrices.layerSegmentation = cv::Mat::zeros(size, CV_8UC1);
        data.matrices.layerEdgeDistance = cv::Mat::zeros(size, CV_32FC1);
        data.matrices.layerScore = cv::Mat::zeros(size, CV_32FC1);
        const bool debugLayersEnabled = data.matrices.debugLayersEnabled;

        const float layerAverageDepth = segment_foreground_and_get_average_depth(data);

        if (layerAverageDepth == 0.0f || data.matrices.layerSegmentation.empty())
        {
            return INVALID_POINT;
        }

        cv::Mat& matScore = data.matrices.layerScore;

        calculate_edge_distance(data.matrices.layerSegmentation,
                                data.matrices.areaSqrt,
                                data.matrices.layerEdgeDistance,
                                data.settings.targetEdgeDistance);

        calculate_integral_area(data.matrices);

        calculate_layer_score(data, layerAverageDepth);

        double min, max;
        cv::Point minLoc, maxLoc;

        cv::minMaxLoc(matScore, &min, &max, &minLoc, &maxLoc, data.matrices.layerSegmentation);

        bool foundPoint = maxLoc.x != -1 && maxLoc.y != -1;

        if (foundPoint)
        {
            bool passesTests = test_single_point(data, maxLoc);

            if (!passesTests)
            {
                //our initial point failed the tests, so now we will test all the points
                //and find the max score with the testPassMap

                ForegroundStatus status = create_test_pass_from_foreground(data);
                if (status == FOREGROUND_EMPTY)
                {
                    foundPoint = false;
                }
                else
                {
                    cv::minMaxLoc(matScore, &min, &max, &minLoc, &maxLoc, data.matrices.layerTestPassMap);
                }
            }
        }

        if (debugLayersEnabled)
        {
            ++data.matrices.layerCount;

            cv::Mat layerCountMat = cv::Mat(size, CV_8UC1, cv::Scalar(data.matrices.layerCount));

            cv::bitwise_or(layerCountMat,
                data.matrices.debugSegmentation,
                data.matrices.debugSegmentation,
                data.matrices.layerSegmentation);

            cv::bitwise_or(layerCountMat,
                data.matrices.debugTestPassMap,
                data.matrices.debugTestPassMap,
                data.matrices.layerTestPassMap);

            cv::Mat scoreMask;
            cv::inRange(matScore, 1, INT_MAX, scoreMask);
            matScore.copyTo(data.matrices.debugScoreValue, scoreMask);
            cv::normalize(matScore, data.matrices.debugScore, 0, 1, cv::NORM_MINMAX, -1, scoreMask);
        }

        if (!foundPoint)
        {
            return INVALID_POINT;
        }

        return maxLoc;
    }

    bool find_next_velocity_seed_pixel(cv::Mat& velocitySignalMatrix,
                                        cv::Mat& searchedMatrix,
                                        cv::Point& foregroundPosition,
                                        cv::Point& nextSearchStart)
    {
        PROFILE_FUNC();
        assert(velocitySignalMatrix.cols == searchedMatrix.cols);
        assert(velocitySignalMatrix.rows == searchedMatrix.rows);
        int width = velocitySignalMatrix.cols;
        int height = velocitySignalMatrix.rows;

        const int startX = MAX(0, MIN(width - 1, nextSearchStart.x));
        const int startY = MAX(0, MIN(height - 1, nextSearchStart.y));

        for (int y = startY; y < height; y++)
        {
            for (int x = startX; x < width; x++)
            {
                uint8_t velocitySignal = *velocitySignalMatrix.ptr<uint8_t>(y, x);
                uint8_t searched = *searchedMatrix.ptr<uint8_t>(y, x);
                if (velocitySignal == PixelType::Foreground && searched != PixelType::Searched)
                {
                    foregroundPosition.x = x;
                    foregroundPosition.y = y;

                    nextSearchStart.x = x + 1;
                    if (nextSearchStart.x < width)
                    {
                        nextSearchStart.y = y;
                    }
                    else
                    {
                        nextSearchStart.x = 0;
                        nextSearchStart.y = y + 1;
                        if (nextSearchStart.y >= height)
                        {
                            return false;
                        }
                    }
                    return true;
                }
            }
        }
        foregroundPosition = segmentation::INVALID_POINT;
        nextSearchStart.x = width;
        nextSearchStart.y = height;
        return false;
    }

    void calculate_edge_distance(cv::Mat& segmentationMatrix,
                                 cv::Mat& areaSqrtMatrix,
                                 cv::Mat& edgeDistanceMatrix,
                                 const float maxEdgeDistance)
    {
        PROFILE_FUNC();
        cv::Mat eroded;
        cv::Mat crossElement = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

        edgeDistanceMatrix = cv::Mat::zeros(segmentationMatrix.size(), CV_32FC1);
        cv::Mat ones = cv::Mat::ones(segmentationMatrix.size(), CV_32FC1);
        segmentationMatrix.copyTo(eroded);

        //close small holes
        int dilateCount = 1;
        for (int i = 0; i < dilateCount; i++)
        {
            cv::dilate(eroded, eroded, crossElement);
        }

        int nonZeroCount = 0;
        const int imageLength = eroded.cols * eroded.rows;
        int iterations = 0;
        const int maxIterations = segmentationMatrix.cols / 2;
        bool done;
        do
        {
            PROFILE_BEGIN(edge_dist_loop);
            //erode makes the image smaller
            cv::erode(eroded, eroded, crossElement);
            //accumulate the eroded image to the edgeDistance buffer
            cv::add(areaSqrtMatrix, edgeDistanceMatrix, edgeDistanceMatrix, eroded, CV_32FC1);

            nonZeroCount = cv::countNonZero(eroded);
            done = (nonZeroCount == 0);
            double min, max;

            cv::minMaxLoc(edgeDistanceMatrix, &min, &max);
            if (max > maxEdgeDistance)
            {
                done = true;
            }

            PROFILE_END();
            //nonZeroCount < imageLength guards against segmentation with all 1's, which will never erode
        } while (!done && nonZeroCount < imageLength && ++iterations < maxIterations);
    }

    void get_circumference_points(cv::Mat& matDepth,
                                  const cv::Point& center,
                                  const float& radius,
                                  const ScalingCoordinateMapper& mapper,
                                  std::vector<astra::Vector2i>& points)
    {
        PROFILE_FUNC();

        int width = matDepth.cols;
        int height = matDepth.rows;
        if (center.x < 0 || center.x >= width ||
            center.y < 0 || center.y >= height ||
            radius < 1)
        {
            return;
        }

        float referenceDepth = matDepth.at<float>(center);
        if (referenceDepth == 0)
        {
            return;
        }

        cv::Point offsetRight = mapper.offset_pixel_location_by_mm(center, radius, 0, referenceDepth);

        //http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
        int pixelRadius = offsetRight.x - center.x;
        int cx = center.x;
        int cy = center.y;

        std::vector<astra::Vector2i> offsets;
        //reserve a slight overestimation of number of points for 1/8 of circumference
        offsets.reserve(pixelRadius);

        {
            int dx = pixelRadius; //radius in pixels
            int dy = 0;
            int radiusError = 1 - dx;

            while (dx >= dy)
            {
                offsets.push_back(Vector2i(dx, dy));

                dy++;
                if (radiusError < 0)
                {
                    radiusError += 2 * dy + 1;
                }
                else
                {
                    dx--;
                    radiusError += 2 * (dy - dx) + 1;
                }
            }
        }

        //PROFILE_BEGIN(circ_checks);

        //clear & reuse capacity across calls
        points.clear();
        points.reserve(static_cast<int>(pixelRadius * 2.0f * PI_F));

        int length = offsets.size();

        //Order and the permutations of dx,dy are critical here
        //so the points list will contain the points in order

        for (int i = 1; i < length; ++i)
        {
            //dx, dy
            const astra::Vector2i delta = offsets[i];
            const int x = cx + delta.x;
            const int y = cy + delta.y;

            if (x >= 0 && x < width &&
                y >= 0 && y < height)
            {
                points.push_back(astra::Vector2i(x, y));
            }
        }

        //even quadrants are reversed order
        for (int i = length-1; i >= 0; --i)
        {
            //dy, dx
            const astra::Vector2i delta = offsets[i];

            const int dx = delta.x;
            const int dy = delta.y;
            if (dx != dy)
            {
                const int x = cx + dy;
                const int y = cy + dx;
                if (x >= 0 && x < width &&
                    y >= 0 && y < height)
                {
                    points.push_back(astra::Vector2i(x, y));
                }
            }
        }

        for (int i = 1; i < length; ++i)
        {
            //-dy, dx
            const astra::Vector2i delta = offsets[i];
            const int x = cx - delta.y;
            const int y = cy + delta.x;

            if (x >= 0 && x < width &&
                y >= 0 && y < height)
            {
                points.push_back(astra::Vector2i(x, y));
            }
        }

        for (int i = length-1; i >= 0; --i)
        {
            //-dx, dy
            const astra::Vector2i delta = offsets[i];

            const int dx = delta.x;
            const int dy = delta.y;
            if (dx != dy)
            {
                const int x = cx - dx;
                const int y = cy + dy;
                if (x >= 0 && x < width &&
                    y >= 0 && y < height)
                {
                    points.push_back(astra::Vector2i(x, y));
                }
            }
        }

        for (int i = 1; i < length; ++i)
        {
            //-dx, -dy
            const astra::Vector2i delta = offsets[i];
            const int x = cx - delta.x;
            const int y = cy - delta.y;

            if (x >= 0 && x < width &&
                y >= 0 && y < height)
            {
                points.push_back(astra::Vector2i(x, y));
            }
        }

        for (int i = length-1; i >= 0; --i)
        {
            //-dy, -dx
            const astra::Vector2i delta = offsets[i];

            const int dx = delta.x;
            const int dy = delta.y;
            if (dx != dy)
            {
                const int x = cx - dy;
                const int y = cy - dx;
                if (x >= 0 && x < width &&
                    y >= 0 && y < height)
                {
                    points.push_back(astra::Vector2i(x, y));
                }
            }
        }

        for (int i = 1; i < length; ++i)
        {
            //dy, -dx
            const astra::Vector2i delta = offsets[i];
            const int x = cx + delta.y;
            const int y = cy - delta.x;

            if (x >= 0 && x < width &&
                y >= 0 && y < height)
            {
                points.push_back(astra::Vector2i(x, y));
            }
        }

        for (int i = length-1; i >= 0; --i)
        {
            //dx, -dy
            const astra::Vector2i delta = offsets[i];

            const int dx = delta.x;
            const int dy = delta.y;
            if (dx != dy)
            {
                const int x = cx + dx;
                const int y = cy - dy;
                if (x >= 0 && x < width &&
                    y >= 0 && y < height)
                {
                    points.push_back(astra::Vector2i(x, y));
                }
            }
        }
        //PROFILE_END();
    }

    float get_max_sequential_circumference_percentage(cv::Mat& matDepth,
                                                      cv::Mat& matSegmentation,
                                                      const cv::Point& center,
                                                      const float& radius,
                                                      const ScalingCoordinateMapper& mapper,
                                                      std::vector<astra::Vector2i>& points)
    {
        PROFILE_FUNC();
        int foregroundCount = 0;
        int maxCount = 0;
        int firstSegmentCount = 0;
        bool isFirstSegment = true;
        int totalCount = 0;
        bool firstIsForeground = false;
        bool lastIsForeground = false;

        get_circumference_points(matDepth, center, radius, mapper, points);

        for (auto p : points)
        {
            bool isForeground = matSegmentation.at<uint8_t>(p.y, p.x) == PixelType::Foreground;
            if (isForeground)
            {
                ++foregroundCount;
                if (totalCount == 0)
                {
                    //started on a foreground segment, might need to add to the last segment
                    firstIsForeground = true;
                }
            }
            else
            {
                if (foregroundCount > maxCount)
                {
                    maxCount = foregroundCount;
                }
                if (isFirstSegment)
                {
                    firstSegmentCount = foregroundCount;
                    isFirstSegment = false;
                }
                foregroundCount = 0;
            }
            lastIsForeground = isForeground;
            ++totalCount;
        }

        if (lastIsForeground)
        {
            if (firstIsForeground)
            {
                //add the first and last segment
                foregroundCount += firstSegmentCount;
            }

            if (foregroundCount > maxCount)
            {
                maxCount = foregroundCount;
            }
        }

        float percentForeground = maxCount / static_cast<float>(totalCount);

        return percentForeground;
    }

    float count_neighborhood_area(cv::Mat& matSegmentation,
                                  cv::Mat& matDepth,
                                  cv::Mat& matArea,
                                  const cv::Point& center,
                                  const float bandwidth,
                                  const float bandwidthDepth,
                                  const ScalingCoordinateMapper& mapper)
    {
        PROFILE_FUNC();
        if (center.x < 0 || center.y < 0 ||
            center.x >= matDepth.cols || center.y >= matDepth.rows)
        {
            return 0;
        }

        float startingDepth = matDepth.at<float>(center);

        cv::Point topLeft = mapper.offset_pixel_location_by_mm(center, -bandwidth, bandwidth, startingDepth);

        int offsetX = center.x - topLeft.x;
        int offsetY = center.y - topLeft.y;
        cv::Point bottomRight(center.x + offsetX, center.y + offsetY);

        int32_t x0 = MAX(0, topLeft.x);
        int32_t y0 = MAX(0, topLeft.y);
        int32_t x1 = MIN(matDepth.cols - 1, bottomRight.x);
        int32_t y1 = MIN(matDepth.rows - 1, bottomRight.y);

        float area = 0;

        for (int y = y0; y <= y1; y++)
        {
            char* segmentationRow = matSegmentation.ptr<char>(y);
            float* areaRow = matArea.ptr<float>(y);

            segmentationRow += x0;
            areaRow += x0;
            for (int x = x0; x <= x1; ++x, ++areaRow, ++segmentationRow)
            {
                if (*segmentationRow == PixelType::Foreground)
                {
                    area += *areaRow;
                }
            }
        }

        return area;
    }


    float count_neighborhood_area_integral(cv::Mat& matDepth,
                                           cv::Mat& matAreaIntegral,
                                           const cv::Point& center,
                                           const float bandwidth,
                                           const ScalingCoordinateMapper& mapper)
    {
        //PROFILE_FUNC();
        int width = matDepth.cols;
        int height = matDepth.rows;
        if (center.x < 0 || center.y < 0 ||
            center.x >= width || center.y >= height)
        {
            return 0;
        }

        float startingDepth = matDepth.at<float>(center);

        cv::Point topLeft = mapper.offset_pixel_location_by_mm(center, -bandwidth, bandwidth, startingDepth);

        int offsetX = center.x - topLeft.x;
        int offsetY = center.y - topLeft.y;
        cv::Point bottomRight(center.x + offsetX, center.y + offsetY);

        //subtract one from topLeft because formula below is exclusive on the lower bounds
        int32_t x0 = MAX(0, topLeft.x-1);
        int32_t y0 = MAX(0, topLeft.y-1);
        int32_t x1 = MIN(width - 1, bottomRight.x);
        int32_t y1 = MIN(height - 1, bottomRight.y);

        float area = 0;

        area += matAreaIntegral.at<float>(y1, x1);
        area += matAreaIntegral.at<float>(y0, x0);
        area -= matAreaIntegral.at<float>(y0, x1);
        area -= matAreaIntegral.at<float>(y1, x0);

        return area;
    }

}}}}
