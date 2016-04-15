#include "TrackedPoint.h"
#include "PointProcessor.h"
#include "Segmentation.h"
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

namespace astra { namespace plugins { namespace hand {

    PointProcessor::PointProcessor(PointProcessorSettings& settings) :
        m_settings(settings)
    {
        PROFILE_FUNC();
    }

    PointProcessor::~PointProcessor()
    {
        PROFILE_FUNC();
    }

    void PointProcessor::calculate_area(TrackingMatrices& matrices, ScalingCoordinateMapper mapper)
    {
        PROFILE_FUNC();

        const astra::Vector3f* fullSizeWorldPoints = matrices.fullSizeWorldPoints;
        astra::Vector3f* worldPoints = matrices.worldPoints;
        auto depthToWorldData = matrices.depthToWorldData;
        cv::Mat& areaMatrix = matrices.area;
        cv::Mat& areaSqrtMatrix = matrices.areaSqrt;

        cv::Size depthSize = matrices.depth.size();

        areaMatrix = cv::Mat::zeros(depthSize, CV_32FC1);
        areaSqrtMatrix = cv::Mat::zeros(depthSize, CV_32FC1);

        int fullSizeWidth = matrices.depthFullSize.cols;
        int width = depthSize.width;
        int height = depthSize.height;

        float offsetX = mapper.offsetX();
        float offsetY = mapper.offsetY();
        float scale = mapper.scale();
        int intScale = static_cast<int>(scale);

        for (int y = 0; y < height; ++y)
        {
            float* areaRow = areaMatrix.ptr<float>(y);
            float* areaSqrtRow = areaSqrtMatrix.ptr<float>(y);

            for (int x = 0; x < width; ++x, ++worldPoints, ++areaRow, ++areaSqrtRow)
            {
                int fullSizeIndex = (x + y * fullSizeWidth) * intScale;
                const Vector3f& p = fullSizeWorldPoints[fullSizeIndex];
                *worldPoints = p;
                const float depth = p.z;

                if (depth != 0)
                {
                    float depthX = (x + 1 + offsetX) * scale;
                    float depthY = (y + 1 + offsetY) * scale;
                    float normalizedX = depthX / depthToWorldData.resolutionX - .5f;
                    float normalizedY = .5f - depthY / depthToWorldData.resolutionY;

                    float wx = normalizedX * depth * depthToWorldData.xzFactor;
                    float wy = normalizedY * depth * depthToWorldData.yzFactor;

                    float deltaX = wx - p.x;
                    float deltaY = wy - p.y;

                    float area = fabs(deltaX * deltaY);

                    *areaRow = area;
                    *areaSqrtRow = sqrt(area);
                }
                else
                {
                    *areaRow = 0;
                    *areaSqrtRow = 0;
                }
            }
        }
    }

    void PointProcessor::initialize_common_calculations(TrackingMatrices& matrices)
    {
        PROFILE_FUNC();
        auto scalingMapper = get_scaling_mapper(matrices);

        calculate_area(matrices, scalingMapper);
    }

    void PointProcessor::updateTrackedPoints(TrackingMatrices& matrices)
    {
        PROFILE_FUNC();
        auto scalingMapper = get_scaling_mapper(matrices);

        //give priority updates to active points
        for (auto iter = m_trackedPoints.begin(); iter != m_trackedPoints.end(); ++iter)
        {
            TrackedPoint& trackedPoint = *iter;
            if (trackedPoint.pointType == TrackedPointType::ActivePoint)
            {
                updateTrackedPoint(matrices, scalingMapper, trackedPoint);
            }
        }

        int numUpdatedPoints = 0;
        for (auto iter = m_trackedPoints.begin(); iter != m_trackedPoints.end(); ++iter)
        {
            TrackedPoint& trackedPoint = *iter;
            if (trackedPoint.pointType != TrackedPointType::ActivePoint)
            {
                updateTrackedPoint(matrices, scalingMapper, trackedPoint);
            }
            ++numUpdatedPoints;
            if (numUpdatedPoints > m_settings.maxHandPointUpdatesPerFrame)
            {
                break;
            }
        }
    }

    void PointProcessor::updateTrackedPoint(TrackingMatrices& matrices,
                                            ScalingCoordinateMapper& scalingMapper,
                                            TrackedPoint& trackedPoint)
    {
        PROFILE_FUNC();
        const float width = matrices.depth.cols;
        const float height = matrices.depth.rows;

        ++trackedPoint.inactiveFrameCount;

        TrackingData updateTrackingData(matrices,
                                        trackedPoint.position,
                                        trackedPoint.worldPosition,
                                        trackedPoint.referenceAreaSqrt,
                                        VELOCITY_POLICY_IGNORE,
                                        m_settings.segmentationSettings,
                                        TEST_PHASE_UPDATE);

        cv::Point newTargetPoint = segmentation::track_point_from_seed(updateTrackingData);

        validateAndUpdateTrackedPoint(matrices, scalingMapper, trackedPoint, newTargetPoint);

        //lost a tracked point, try to guess the position using previous position delta for second chance to recover

        auto xyDelta = trackedPoint.worldDeltaPosition;
        xyDelta.z = 0;
        double xyDeltaNorm = cv::norm(xyDelta);
        if (trackedPoint.trackingStatus != TrackingStatus::Tracking &&
            newTargetPoint == segmentation::INVALID_POINT &&
            xyDeltaNorm > m_settings.secondChanceMinDistance)
        {
            auto movementDirection = xyDelta * (1.0f / xyDeltaNorm);
            float maxSegmentationDist = m_settings.segmentationSettings.maxSegmentationDist;
            auto estimatedWorldPosition = trackedPoint.worldPosition + movementDirection * maxSegmentationDist;

            cv::Point3f estimatedPosition = scalingMapper.convert_world_to_depth(estimatedWorldPosition);

            cv::Point seedPosition;
            seedPosition.x = MAX(0, MIN(width - 1, static_cast<int>(estimatedPosition.x)));
            seedPosition.y = MAX(0, MIN(height - 1, static_cast<int>(estimatedPosition.y)));

            TrackingData recoverTrackingData(matrices,
                                             seedPosition,
                                             estimatedWorldPosition,
                                             trackedPoint.referenceAreaSqrt,
                                             VELOCITY_POLICY_IGNORE,
                                             m_settings.segmentationSettings,
                                             TEST_PHASE_UPDATE);

            newTargetPoint = segmentation::track_point_from_seed(recoverTrackingData);

            //test for invalid point here so we don't increment failed test counts
            //for second chance recovery
            if (newTargetPoint != segmentation::INVALID_POINT)
            {
                validateAndUpdateTrackedPoint(matrices, scalingMapper, trackedPoint, newTargetPoint);
            }

            if (trackedPoint.trackingStatus == TrackingStatus::Tracking)
            {
                LOG_TRACE("PointProcessor", "updateTrackedPoint 2nd chance recovered #%d",
                              trackedPoint.trackingId);
            }
        }
    }

    void PointProcessor::reset()
    {
        PROFILE_FUNC();
        m_trackedPoints.clear();
        m_nextTrackingId = 0;
    }

    void PointProcessor::update_full_resolution_points(TrackingMatrices& matrices)
    {
        PROFILE_FUNC();
        for (auto iter = m_trackedPoints.begin(); iter != m_trackedPoints.end(); ++iter)
        {
            TrackedPoint& trackedPoint = *iter;

            float resizeFactor = get_resize_factor(matrices);

            //add 0.5 to center on the middle of the pixel
            trackedPoint.fullSizePosition.x = (trackedPoint.position.x + 0.5) * resizeFactor;
            trackedPoint.fullSizePosition.y = (trackedPoint.position.y + 0.5) * resizeFactor;

            bool resizeNeeded = matrices.depthFullSize.cols != matrices.depth.cols;

            bool processRefinedPosition = false;

            if (resizeNeeded &&
                trackedPoint.trackingStatus == TrackingStatus::Tracking &&
                trackedPoint.pointType == TrackedPointType::ActivePoint)
            {
                cv::Point3f refinedWorldPosition = trackedPoint.worldPosition;
                if (processRefinedPosition)
                {
                    refinedWorldPosition = get_refined_high_res_position(matrices, trackedPoint);
                }

                cv::Point3f smoothedWorldPosition = smooth_world_positions(trackedPoint.fullSizeWorldPosition, refinedWorldPosition);

                update_tracked_point_from_world_position(trackedPoint,
                                                         smoothedWorldPosition,
                                                         resizeFactor,
                                                         matrices.depthToWorldData);
            }
            else
            {
                trackedPoint.fullSizeWorldPosition = trackedPoint.worldPosition;
                trackedPoint.fullSizeWorldDeltaPosition = trackedPoint.worldDeltaPosition;
            }
        }
    }

    void PointProcessor::update_trajectories()
    {
        PROFILE_FUNC();
        for (auto iter = m_trackedPoints.begin(); iter != m_trackedPoints.end(); ++iter)
        {
            //TODO take this and make it a method on TrackedPoint
            TrackedPoint& trackedPoint = *iter;
            int trackingId = trackedPoint.trackingId;
            auto it = m_trajectories.find(trackingId);

            if (it == m_trajectories.end())
            {
                TrajectoryAnalyzer analyzer(trackingId, m_settings.trajectoryAnalyzerSettings);
                analyzer.update(trackedPoint);
                m_trajectories.insert(std::make_pair(trackingId, analyzer));
            }
            else
            {
                TrajectoryAnalyzer& analyzer = it->second;
                analyzer.update(trackedPoint);

                if (analyzer.isWaveGesture())
                {
                    end_probation(trackedPoint);
                    trackedPoint.pointType = TrackedPointType::ActivePoint;
                }
            }
        }
    }

    cv::Point3f PointProcessor::get_refined_high_res_position(TrackingMatrices& matrices,
                                                            const TrackedPoint& trackedPoint)
    {
        PROFILE_FUNC();
        assert(trackedPoint.pointType == TrackedPointType::ActivePoint);

        if (trackedPoint.worldPosition.z == 0)
        {
            return trackedPoint.worldPosition;
        }

        int fullWidth = matrices.depthFullSize.cols;
        int fullHeight = matrices.depthFullSize.rows;
        int processingWidth = matrices.depth.cols;
        int processingHeight = matrices.depth.rows;

        int fullSizeX = trackedPoint.fullSizePosition.x;
        int fullSizeY = trackedPoint.fullSizePosition.y;
        int windowLeft = MAX(0, MIN(fullWidth - processingWidth, fullSizeX - processingWidth / 2));
        int windowTop = MAX(0, MIN(fullHeight - processingHeight, fullSizeY - processingHeight / 2));

        cv::Point roiPosition(fullSizeX - windowLeft, fullSizeY - windowTop);

        float referenceAreaSqrt = matrices.areaSqrt.at<float>(roiPosition);
        if (referenceAreaSqrt == 0)
        {
            return trackedPoint.worldPosition;
        }

        //Create a window into the full size data
        cv::Mat roi = matrices.depthFullSize(cv::Rect(windowLeft, windowTop, processingWidth, processingHeight));
        //copyTo for now so .at works with local coords in functions that use it
        roi.copyTo(matrices.depth);

        //initialize_common_calculations(matrices);
        ScalingCoordinateMapper roiMapper(matrices.depthToWorldData, 1.0, windowLeft, windowTop);

        calculate_area(matrices, roiMapper);

        TrackingData refinementTrackingData(matrices,
                                            roiPosition,
                                            trackedPoint.worldPosition,
                                            referenceAreaSqrt,
                                            VELOCITY_POLICY_IGNORE,
                                            m_settings.segmentationSettings,
                                            TEST_PHASE_UPDATE);

        cv::Point targetPoint = segmentation::track_point_from_seed(refinementTrackingData);

        if (targetPoint == segmentation::INVALID_POINT)
        {
            return trackedPoint.worldPosition;
        }

        int refinedFullSizeX = targetPoint.x + windowLeft;
        int refinedFullSizeY = targetPoint.y + windowTop;

        float refinedDepth = matrices.depthFullSize.at<float>(refinedFullSizeY, refinedFullSizeX);

        if (refinedDepth == 0)
        {
            refinedDepth = trackedPoint.worldPosition.z;
        }

        cv::Point3f refinedWorldPosition = cv_convert_depth_to_world(matrices.depthToWorldData,
                                                                     refinedFullSizeX,
                                                                     refinedFullSizeY,
                                                                     refinedDepth);

        return refinedWorldPosition;
    }

    cv::Point3f PointProcessor::smooth_world_positions(const cv::Point3f& oldWorldPosition,
                                                       const cv::Point3f& newWorldPosition)
    {
        PROFILE_FUNC();
        float smoothingFactor = m_settings.pointSmoothingFactor;

        float delta = cv::norm(newWorldPosition - oldWorldPosition);
        if (delta < m_settings.pointSmoothingDeadZone)
        {
            float factorRamp = delta / m_settings.pointSmoothingDeadZone;
            smoothingFactor = m_settings.pointSmoothingFactor * factorRamp +
                              m_settings.pointDeadBandSmoothingFactor * (1 - factorRamp);
        }

        return oldWorldPosition * (1 - smoothingFactor) + newWorldPosition * smoothingFactor;
    }

    void PointProcessor::update_tracked_point_from_world_position(TrackedPoint& trackedPoint,
                                                                  const cv::Point3f& newWorldPosition,
                                                                  const float resizeFactor,
                                                                  const conversion_cache_t& depthToWorldData)
    {
        PROFILE_FUNC();
        cv::Point3f fullSizeDepthPosition = cv_convert_world_to_depth(depthToWorldData, newWorldPosition);

        trackedPoint.fullSizePosition = cv::Point(fullSizeDepthPosition.x, fullSizeDepthPosition.y);

        cv::Point3f deltaPosition = newWorldPosition - trackedPoint.fullSizeWorldPosition;
        trackedPoint.fullSizeWorldPosition = newWorldPosition;
        trackedPoint.fullSizeWorldDeltaPosition = deltaPosition;
    }

    void PointProcessor::start_probation(TrackedPoint& trackedPoint)
    {
        PROFILE_FUNC();
        if (!trackedPoint.isInProbation)
        {
            LOG_TRACE("PointProcessor", "started probation for: %d", trackedPoint.trackingId);
            trackedPoint.isInProbation = true;
            trackedPoint.probationFrameCount = 0;
            trackedPoint.failedTestCount = 0;
        }
    }

    void PointProcessor::end_probation(TrackedPoint& trackedPoint)
    {
        PROFILE_FUNC();
        trackedPoint.isInProbation = false;
        trackedPoint.failedTestCount = 0;
    }

    void PointProcessor::update_tracked_point_data(TrackingMatrices& matrices, ScalingCoordinateMapper& scalingMapper, TrackedPoint& trackedPoint, const cv::Point& newTargetPoint)
    {
        PROFILE_FUNC();
        float depth = matrices.depth.at<float>(newTargetPoint);
        cv::Point3f worldPosition = scalingMapper.convert_depth_to_world(newTargetPoint.x, newTargetPoint.y, depth);

        cv::Point3f deltaPosition = worldPosition - trackedPoint.worldPosition;
        trackedPoint.worldPosition = worldPosition;
        trackedPoint.worldDeltaPosition = deltaPosition;

        trackedPoint.position = newTargetPoint;
        trackedPoint.referenceAreaSqrt = matrices.areaSqrt.at<float>(trackedPoint.position);

        auto steadyDist = cv::norm(worldPosition - trackedPoint.steadyWorldPosition);

        if (steadyDist > m_settings.steadyDeadBandRadius)
        {
            trackedPoint.steadyWorldPosition = worldPosition;
            trackedPoint.inactiveFrameCount = 0;
        }
    }

    void PointProcessor::validateAndUpdateTrackedPoint(TrackingMatrices& matrices,
                                                       ScalingCoordinateMapper& scalingMapper,
                                                       TrackedPoint& trackedPoint,
                                                       const cv::Point& newTargetPoint)
    {
        PROFILE_FUNC();
        if(trackedPoint.trackingStatus == TrackingStatus::Dead)
        {
            return;
        }

        auto oldStatus = trackedPoint.trackingStatus;
        const TestBehavior outputTestLog = TEST_BEHAVIOR_NONE;

        bool validPointInRange = segmentation::test_point_in_range(matrices,
                                                                   newTargetPoint,
                                                                   outputTestLog);

        if (validPointInRange)
        {
            trackedPoint.trackingStatus = TrackingStatus::Tracking;
            if (trackedPoint.pointType == TrackedPointType::ActivePoint)
            {
                trackedPoint.failedTestCount = 0;
            }
        }
        else
        {
            start_probation(trackedPoint);
            ++trackedPoint.failedTestCount;
        }

        if (trackedPoint.isInProbation)
        {
            bool probationFailed = false;
            if (trackedPoint.pointType == TrackedPointType::ActivePoint &&
                trackedPoint.failedTestCount >= m_settings.maxFailedTestsInProbationActivePoints)
            {
                //had valid in range points but must have failed the real tests

                //failed N consecutive tests within the probation period
                //gave the active point a few extra frames to recover
                trackedPoint.trackingStatus = TrackingStatus::Lost;
                probationFailed = true;
                LOG_TRACE("PointProcessor", "lost an active point: %d", trackedPoint.trackingId);
            }
            else if (trackedPoint.pointType == TrackedPointType::CandidatePoint &&
                     trackedPoint.failedTestCount >= m_settings.maxFailedTestsInProbation)
            {
                //failed N tests total (non-consecutive) within the probation period
                //too many failed tests, so long...
                trackedPoint.trackingStatus = TrackingStatus::Lost;
                probationFailed = true;
            }

            ++trackedPoint.probationFrameCount;
            if (trackedPoint.probationFrameCount > m_settings.probationFrameCount || probationFailed)
            {
                //you're out of probation, but we're keeping an eye on you...
                end_probation(trackedPoint);
                LOG_TRACE("PointProcessor", "ended probation: %d count: %d/%d probationFailed: %d",
                            trackedPoint.trackingId,
                            trackedPoint.probationFrameCount,
                            m_settings.probationFrameCount,
                            probationFailed);
            }
        }

        if (validPointInRange)
        {
            update_tracked_point_data(matrices, scalingMapper, trackedPoint, newTargetPoint);
        }

        if (trackedPoint.trackingStatus != oldStatus)
        {
            LOG_TRACE("PointProcessor", "validateAndUpdateTrackedPoint: #%d status %s --> status %s",
                           trackedPoint.trackingId,
                           tracking_status_to_string(oldStatus).c_str(),
                           tracking_status_to_string(trackedPoint.trackingStatus).c_str());
        }
    }

    void PointProcessor::removeDuplicatePoints()
    {
        PROFILE_FUNC();

        for (auto iter = m_trackedPoints.begin(); iter != m_trackedPoints.end(); ++iter)
        {
            TrackedPoint& tracked = *iter;
            for (auto otherIter = m_trackedPoints.begin(); otherIter != m_trackedPoints.end(); ++otherIter)
            {
                TrackedPoint& otherTracked = *otherIter;
                bool bothNotDead = tracked.trackingStatus != TrackingStatus::Dead && otherTracked.trackingStatus != TrackingStatus::Dead;
                float pointDist = cv::norm(tracked.worldPosition - otherTracked.worldPosition);
                if (tracked.trackingId != otherTracked.trackingId &&
                    bothNotDead &&
                    pointDist < m_settings.mergePointDistance)
                {
                    tracked.inactiveFrameCount = MIN(tracked.inactiveFrameCount, otherTracked.inactiveFrameCount);
                    if (otherTracked.pointType == TrackedPointType::ActivePoint && tracked.pointType != TrackedPointType::ActivePoint)
                    {
                        tracked.trackingId = otherTracked.trackingId;
                        tracked.pointType = TrackedPointType::ActivePoint;
                    }
                    otherTracked.trackingStatus = TrackingStatus::Dead;
                }
            }
        }
    }

    void PointProcessor::removeOldOrDeadPoints()
    {
        PROFILE_FUNC();
        for (auto iter = m_trackedPoints.begin(); iter != m_trackedPoints.end();)
        {
            TrackedPoint& tracked = *iter;

            int max = m_settings.maxInactiveFramesForCandidatePoints;
            if (tracked.pointType == TrackedPointType::ActivePoint)
            {
                if (tracked.trackingStatus == TrackingStatus::Lost)
                {
                    max = m_settings.maxInactiveFramesForLostPoints;
                }
                else
                {
                    max = m_settings.maxInactiveFramesForActivePoints;
                }
            }
            //if inactive for more than a certain number of frames, or dead, remove point
            if (tracked.inactiveFrameCount > max || tracked.trackingStatus == TrackingStatus::Dead)
            {
                m_trajectories.erase(tracked.trackingId);

                iter = m_trackedPoints.erase(iter);
            }
            else
            {
                ++iter;
            }
        }
    }

    void PointProcessor::updateTrackedPointOrCreateNewPointFromSeedPosition(TrackingMatrices& matrices,
                                                                            const cv::Point& seedPosition)
    {
        PROFILE_FUNC();
        float referenceDepth = matrices.depth.at<float>(seedPosition);
        float referenceAreaSqrt = matrices.areaSqrt.at<float>(seedPosition);
        if (referenceDepth == 0 || referenceAreaSqrt == 0 || seedPosition == segmentation::INVALID_POINT)
        {
            //Cannot expect to properly segment when the seedPosition has zero depth
            return;
        }

        auto scalingMapper = get_scaling_mapper(matrices);
        cv::Point3f referenceWorldPosition =
                        scalingMapper.convert_depth_to_world(seedPosition.x,
                                                             seedPosition.y,
                                                             referenceDepth);

        TrackingData createTrackingData(matrices,
                                        seedPosition,
                                        referenceWorldPosition,
                                        referenceAreaSqrt,
                                        VELOCITY_POLICY_RESET_TTL,
                                        m_settings.segmentationSettings,
                                        TEST_PHASE_CREATE);

        cv::Point targetPoint = segmentation::track_point_from_seed(createTrackingData);

        const TestBehavior outputTestLog = TEST_BEHAVIOR_NONE;

        bool validPointInRange = segmentation::test_point_in_range(matrices,
                                                                   targetPoint,
                                                                   outputTestLog);

        if (!validPointInRange)
        {
            return;
        }

        bool existingPoint = false;

        float depth = matrices.depth.at<float>(targetPoint);

        cv::Point3f worldPosition = scalingMapper.convert_depth_to_world(targetPoint.x,
                                                                         targetPoint.y,
                                                                         depth);

        for (auto iter = m_trackedPoints.begin(); iter != m_trackedPoints.end(); ++iter)
        {
            TrackedPoint& trackedPoint = *iter;
            if (trackedPoint.trackingStatus != TrackingStatus::Dead)
            {
                float dist = cv::norm(trackedPoint.worldPosition - worldPosition);
                float maxDist = m_settings.maxMatchDistDefault;
                bool lostPoint = trackedPoint.trackingStatus == TrackingStatus::Lost;
                if (lostPoint && trackedPoint.pointType == TrackedPointType::ActivePoint)
                {
                    maxDist = m_settings.maxMatchDistLostActive;
                }
                if (dist < maxDist)
                {
                    trackedPoint.inactiveFrameCount = 0;
                    if (lostPoint)
                    {
                        //Recover a lost point -- move it to the recovery position
                        trackedPoint.position = targetPoint;
                        trackedPoint.referenceAreaSqrt = matrices.areaSqrt.at<float>(trackedPoint.position);

                        trackedPoint.worldPosition = worldPosition;
                        trackedPoint.worldDeltaPosition = cv::Point3f();

                        LOG_TRACE("PointProcessor", "createCycle: Recovered #%d",
                                        trackedPoint.trackingId);

                        //it could be faulty recovery, so start out in probation just like a new point
                        start_probation(trackedPoint);
                    }
                    trackedPoint.trackingStatus = TrackingStatus::Tracking;
                    existingPoint = true;
                    break;
                }
            }
        }
        if (!existingPoint)
        {
            LOG_TRACE("PointProcessor", "createCycle: Created new point #%d",
                           m_nextTrackingId);

            TrackedPoint newPoint(targetPoint, worldPosition, m_nextTrackingId);
            newPoint.pointType = TrackedPointType::CandidatePoint;
            newPoint.trackingStatus = TrackingStatus::Tracking;
            ++m_nextTrackingId;
            m_trackedPoints.push_back(newPoint);
            start_probation(newPoint);
        }
    }
}}}
