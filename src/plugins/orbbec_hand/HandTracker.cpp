// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "HandTracker.h"
#include "Segmentation.h"
#include <AstraUL/streams/hand_types.h>
#include <AstraUL/astraul_ctypes.h>
#include <Astra/Plugins/PluginKit.h>
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 
#define PROFILE_BLOCK(...) 
#define PROFILE_UPDATE(...) 
#define PROFILE_OUTPUT(...) 

namespace astra { namespace plugins { namespace hand {

        using namespace std;

        HandTracker::HandTracker(PluginServiceProxy& pluginService,
                                 astra_streamset_t streamSet,
                                 StreamDescription& depthDesc,
                                 HandSettings& settings) :
            m_streamset(get_uri_for_streamset(pluginService, streamSet)),
            m_reader(m_streamset.create_reader()),
            m_depthStream(m_reader.stream<DepthStream>(depthDesc.subtype())),
            m_settings(settings),
            m_pluginService(pluginService),
            m_depthUtility(settings.processingSizeWidth, settings.processingSizeHeight, settings.depthUtilitySettings),
            m_pointProcessor(settings.pointProcessorSettings),
            m_processingSizeWidth(settings.processingSizeWidth),
            m_processingSizeHeight(settings.processingSizeHeight)

        {
            PROFILE_FUNC();

            create_streams(m_pluginService, streamSet);
            m_depthStream.start();

            m_reader.stream<PointStream>().start();
            m_reader.addListener(*this);
        }

        HandTracker::~HandTracker()
        {
            PROFILE_FUNC();
            if (m_worldPoints != nullptr)
            {
                delete[] m_worldPoints;
                m_worldPoints = nullptr;
            }
        }

        void HandTracker::create_streams(PluginServiceProxy& pluginService, astra_streamset_t streamSet)
        {
            PROFILE_FUNC();
            LOG_INFO("HandTracker", "creating hand streams");
            auto hs = make_stream<HandStream>(pluginService, streamSet, ASTRA_HANDS_MAX_HAND_COUNT);
            m_handStream = std::unique_ptr<HandStream>(std::move(hs));

            const int bytesPerPixel = 3;
            auto dhs = make_stream<DebugHandStream>(pluginService,
                                                    streamSet,
                                                    m_processingSizeWidth,
                                                    m_processingSizeHeight,
                                                    bytesPerPixel);
            m_debugImageStream = std::unique_ptr<DebugHandStream>(std::move(dhs));
        }

        void HandTracker::on_frame_ready(StreamReader& reader, Frame& frame)
        {
            PROFILE_FUNC();
            if (m_handStream->has_connections() ||
                m_debugImageStream->has_connections())
            {
                DepthFrame depthFrame = frame.get<DepthFrame>();
                PointFrame pointFrame = frame.get<PointFrame>();
                update_tracking(depthFrame, pointFrame);
            }

            PROFILE_UPDATE();
        }

        void HandTracker::reset()
        {
            PROFILE_FUNC();
            m_depthUtility.reset();
            m_pointProcessor.reset();
        }

        void HandTracker::update_tracking(DepthFrame& depthFrame, PointFrame& pointFrame)
        {
            PROFILE_FUNC();
            if (!m_debugImageStream->pause_input())
            {
                m_depthUtility.processDepthToVelocitySignal(depthFrame, m_matDepth, m_matDepthFullSize, m_matVelocitySignal);
            }

            track_points(m_matDepth, m_matDepthFullSize, m_matVelocitySignal, pointFrame.data());

            //use same frameIndex as source depth frame
            astra_frame_index_t frameIndex = depthFrame.frameIndex();

            if (m_handStream->has_connections())
            {
                generate_hand_frame(frameIndex);
            }

            if (m_debugImageStream->has_connections())
            {
                generate_hand_debug_image_frame(frameIndex);
            }
        }

        void HandTracker::track_points(cv::Mat& matDepth,
                                       cv::Mat& matDepthFullSize,
                                       cv::Mat& matVelocitySignal,
                                       const Vector3f* fullSizeWorldPoints)
        {
            PROFILE_FUNC();

            m_layerSegmentation = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_layerScore = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_layerEdgeDistance = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_debugUpdateSegmentation = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_debugCreateSegmentation = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_debugRefineSegmentation = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_updateForegroundSearched = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_createForegroundSearched = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_refineForegroundSearched = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_debugUpdateScore = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_debugCreateScore = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_matDepthWindow = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_refineSegmentation = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_refineScore = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_refineEdgeDistance = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_debugUpdateScoreValue = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_debugCreateScoreValue = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_debugRefineScoreValue = cv::Mat::zeros(matDepth.size(), CV_32FC1);
            m_debugCreateTestPassMap = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_debugUpdateTestPassMap = cv::Mat::zeros(matDepth.size(), CV_8UC1);
            m_debugRefineTestPassMap = cv::Mat::zeros(matDepth.size(), CV_8UC1);

            int numPoints = matDepth.cols * matDepth.rows;
            if (m_worldPoints == nullptr || m_numWorldPoints != numPoints)
            {
                if (m_worldPoints != nullptr)
                {
                    delete[] m_worldPoints;
                    m_worldPoints = nullptr;
                }

                m_numWorldPoints = numPoints;
                m_worldPoints = new astra::Vector3f[numPoints];
            }

            const conversion_cache_t depthToWorldData = m_depthStream.depth_to_world_data();

            bool debugLayersEnabled = m_debugImageStream->has_connections();
            bool enabledTestPassMap = m_debugImageStream->view_type() == DEBUG_HAND_VIEW_TEST_PASS_MAP;

            TrackingMatrices updateMatrices(matDepthFullSize,
                                            matDepth,
                                            m_matArea,
                                            m_matAreaSqrt,
                                            matVelocitySignal,
                                            m_updateForegroundSearched,
                                            m_layerSegmentation,
                                            m_layerScore,
                                            m_layerEdgeDistance,
                                            m_layerIntegralArea,
                                            m_layerTestPassMap,
                                            m_debugUpdateSegmentation,
                                            m_debugUpdateScore,
                                            m_debugUpdateScoreValue,
                                            m_debugUpdateTestPassMap,
                                            enabledTestPassMap,
                                            fullSizeWorldPoints,
                                            m_worldPoints,
                                            debugLayersEnabled,
                                            m_depthStream.coordinateMapper(),
                                            depthToWorldData);

            if (!m_debugImageStream->pause_input())
            {
                m_pointProcessor.initialize_common_calculations(updateMatrices);
            }

            //Update existing points first so that if we lose a point, we might recover it in the "add new" stage below
            //without having at least one frame of a lost point.

            m_pointProcessor.updateTrackedPoints(updateMatrices);

            m_pointProcessor.removeDuplicatePoints();

            TrackingMatrices createMatrices(matDepthFullSize,
                                            matDepth,
                                            m_matArea,
                                            m_matAreaSqrt,
                                            matVelocitySignal,
                                            m_createForegroundSearched,
                                            m_layerSegmentation,
                                            m_layerScore,
                                            m_layerEdgeDistance,
                                            m_layerIntegralArea,
                                            m_layerTestPassMap,
                                            m_debugCreateSegmentation,
                                            m_debugCreateScore,
                                            m_debugCreateScoreValue,
                                            m_debugCreateTestPassMap,
                                            enabledTestPassMap,
                                            fullSizeWorldPoints,
                                            m_worldPoints,
                                            debugLayersEnabled,
                                            m_depthStream.coordinateMapper(),
                                            depthToWorldData);

            //add new points (unless already tracking)
            if (!m_debugImageStream->use_mouse_probe())
            {
                cv::Point seedPosition;
                cv::Point nextSearchStart(0, 0);
                while (segmentation::find_next_velocity_seed_pixel(matVelocitySignal, m_createForegroundSearched, seedPosition, nextSearchStart))
                {
                    m_pointProcessor.updateTrackedPointOrCreateNewPointFromSeedPosition(createMatrices, seedPosition);
                }
            }
            else
            {
                debug_spawn_point(createMatrices);
            }

            debug_probe_point(createMatrices);

            //remove old points
            m_pointProcessor.removeOldOrDeadPoints();

            TrackingMatrices refinementMatrices(matDepthFullSize,
                                                m_matDepthWindow,
                                                m_matArea,
                                                m_matAreaSqrt,
                                                matVelocitySignal,
                                                m_refineForegroundSearched,
                                                m_refineSegmentation,
                                                m_refineScore,
                                                m_refineEdgeDistance,
                                                m_layerIntegralArea,
                                                m_layerTestPassMap,
                                                m_debugRefineSegmentation,
                                                m_debugRefineScore,
                                                m_debugRefineScoreValue,
                                                m_debugRefineTestPassMap,
                                                enabledTestPassMap,
                                                fullSizeWorldPoints,
                                                m_worldPoints,
                                                false,
                                                m_depthStream.coordinateMapper(),
                                                depthToWorldData);

            m_pointProcessor.update_full_resolution_points(refinementMatrices);

            m_pointProcessor.update_trajectories();
        }

        void HandTracker::debug_probe_point(TrackingMatrices& matrices)
        {
            if (!m_debugImageStream->use_mouse_probe())
            {
                return;
            }

            cv::Point probePosition = get_mouse_probe_position();

            cv::Mat& matDepth = matrices.depth;

            float depth = matDepth.at<float>(probePosition);
            float score = m_debugCreateScoreValue.at<float>(probePosition);
            float edgeDist = m_layerEdgeDistance.at<float>(probePosition);

            auto segmentationSettings = m_settings.pointProcessorSettings.segmentationSettings;

            const TestBehavior outputTestLog = TEST_BEHAVIOR_LOG;
            const TestPhase phase = TEST_PHASE_CREATE;

            bool validPointInRange = segmentation::test_point_in_range(matrices,
                                                                       probePosition,
                                                                       outputTestLog);
            bool validPointArea = false;
            bool validRadiusTest = false;
            bool validNaturalEdges = false;

            if (validPointInRange)
            {
                validPointArea = segmentation::test_point_area_integral(matrices,
                                                                        matrices.layerIntegralArea,
                                                segmentationSettings.areaTestSettings,
                                                                        probePosition,
                                                                        phase,
                                                                        outputTestLog);
                validRadiusTest = segmentation::test_foreground_radius_percentage(matrices,
                                                segmentationSettings.circumferenceTestSettings,
                                                                                  probePosition,
                                                                                  phase,
                                                                                  outputTestLog);

                validNaturalEdges = segmentation::test_natural_edges(matrices,
                                                segmentationSettings.naturalEdgeTestSettings,
                                                                     probePosition,
                                                                     phase,
                                                                     outputTestLog);
            }

            bool allPointsPass = validPointInRange &&
                                 validPointArea &&
                                 validRadiusTest &&
                                 validNaturalEdges;

            LOG_INFO("HandTracker", "depth: %f score: %f edge %f tests: %s",
                       depth,
                       score,
                       edgeDist,
                       allPointsPass ? "PASS" : "FAIL");
        }

        void HandTracker::debug_spawn_point(TrackingMatrices& matrices)
        {
            if (!m_debugImageStream->pause_input())
            {
                m_pointProcessor.initialize_common_calculations(matrices);
            }
            cv::Point seedPosition = get_spawn_position();

            m_pointProcessor.updateTrackedPointOrCreateNewPointFromSeedPosition(matrices, seedPosition);
        }

        cv::Point HandTracker::get_spawn_position()
        {
            auto normPosition = m_debugImageStream->mouse_norm_position();

            if (m_debugImageStream->spawn_point_locked())
            {
                normPosition = m_debugImageStream->spawn_norm_position();
            }

            int x = MAX(0, MIN(m_processingSizeWidth, normPosition.x * m_processingSizeWidth));
            int y = MAX(0, MIN(m_processingSizeHeight, normPosition.y * m_processingSizeHeight));
            return cv::Point(x, y);
        }

        cv::Point HandTracker::get_mouse_probe_position()
        {
            auto normPosition = m_debugImageStream->mouse_norm_position();
            int x = MAX(0, MIN(m_processingSizeWidth, normPosition.x * m_processingSizeWidth));
            int y = MAX(0, MIN(m_processingSizeHeight, normPosition.y * m_processingSizeHeight));
            return cv::Point(x, y);
        }

        void HandTracker::generate_hand_frame(astra_frame_index_t frameIndex)
        {
            PROFILE_FUNC();

            astra_handframe_wrapper_t* handFrame = m_handStream->begin_write(frameIndex);

            if (handFrame != nullptr)
            {
                handFrame->frame.handpoints = reinterpret_cast<astra_handpoint_t*>(&(handFrame->frame_data));
                handFrame->frame.handCount = ASTRA_HANDS_MAX_HAND_COUNT;

                update_hand_frame(m_pointProcessor.get_trackedPoints(), handFrame->frame);

                PROFILE_BEGIN(end_write);
                m_handStream->end_write();
                PROFILE_END();
            }
        }

        void HandTracker::generate_hand_debug_image_frame(astra_frame_index_t frameIndex)
        {
            PROFILE_FUNC();
            astra_imageframe_wrapper_t* debugImageFrame = m_debugImageStream->begin_write(frameIndex);

            if (debugImageFrame != nullptr)
            {
                debugImageFrame->frame.data = reinterpret_cast<uint8_t *>(&(debugImageFrame->frame_data));

                astra_image_metadata_t metadata;

                metadata.width = m_processingSizeWidth;
                metadata.height = m_processingSizeHeight;
                metadata.pixelFormat = astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888;

                debugImageFrame->frame.metadata = metadata;
                update_debug_image_frame(debugImageFrame->frame);

                m_debugImageStream->end_write();
            }
        }

        void HandTracker::update_hand_frame(vector<TrackedPoint>& internalTrackedPoints, _astra_handframe& frame)
        {
            PROFILE_FUNC();
            int handIndex = 0;
            int maxHandCount = frame.handCount;

            bool includeCandidates = m_handStream->include_candidate_points();

            for (auto it = internalTrackedPoints.begin(); it != internalTrackedPoints.end(); ++it)
            {
                TrackedPoint internalPoint = *it;

                TrackingStatus status = internalPoint.trackingStatus;
                TrackedPointType pointType = internalPoint.pointType;

                bool includeByStatus = status == Tracking ||
                                       status == Lost;
                bool includeByType = pointType == ActivePoint ||
                                     (pointType == CandidatePoint && includeCandidates);
                if (includeByStatus && includeByType && handIndex < maxHandCount)
                {
                    astra_handpoint_t& point = frame.handpoints[handIndex];
                    ++handIndex;

                    point.trackingId = internalPoint.trackingId;

                    point.depthPosition.x = internalPoint.fullSizePosition.x;
                    point.depthPosition.y = internalPoint.fullSizePosition.y;

                    copy_position(internalPoint.fullSizeWorldPosition, point.worldPosition);
                    copy_position(internalPoint.fullSizeWorldDeltaPosition, point.worldDeltaPosition);

                    point.status = convert_hand_status(status, pointType);
                }
            }
            for (int i = handIndex; i < maxHandCount; ++i)
            {
                astra_handpoint_t& point = frame.handpoints[i];
                reset_hand_point(point);
            }
        }

        void HandTracker::copy_position(cv::Point3f& source, astra_vector3f_t& target)
        {
            PROFILE_FUNC();
            target.x = source.x;
            target.y = source.y;
            target.z = source.z;
        }

        astra_handstatus_t HandTracker::convert_hand_status(TrackingStatus status, TrackedPointType type)
        {
            PROFILE_FUNC();
            if (type == TrackedPointType::CandidatePoint)
            {
                return HAND_STATUS_CANDIDATE;
            }
            switch (status)
            {
            case Tracking:
                return HAND_STATUS_TRACKING;
                break;
            case Lost:
                return HAND_STATUS_LOST;
                break;
            case Dead:
            case NotTracking:
            default:
                return HAND_STATUS_NOTTRACKING;
                break;
            }
        }

        void HandTracker::reset_hand_point(astra_handpoint_t& point)
        {
            PROFILE_FUNC();
            point.trackingId = -1;
            point.status = HAND_STATUS_NOTTRACKING;
            point.depthPosition = astra_vector2i_t();
            point.worldPosition = astra_vector3f_t();
            point.worldDeltaPosition = astra_vector3f_t();
        }

        void mark_image_pixel(_astra_imageframe& imageFrame,
                              RGBPixel color,
                              astra::Vector2i p)
        {
            PROFILE_FUNC();
            RGBPixel* colorData = static_cast<RGBPixel*>(imageFrame.data);
            int index = p.x + p.y * imageFrame.metadata.width;
            colorData[index] = color;
        }

        void HandTracker::overlay_circle(_astra_imageframe& imageFrame)
        {
            PROFILE_FUNC();

            float resizeFactor = m_matDepthFullSize.cols / static_cast<float>(m_matDepth.cols);
            ScalingCoordinateMapper mapper(m_depthStream.depth_to_world_data(), resizeFactor);

            RGBPixel color(255, 0, 255);

            auto segmentationSettings = m_settings.pointProcessorSettings.segmentationSettings;
            float foregroundRadius1 = segmentationSettings.circumferenceTestSettings.foregroundRadius1;
            float foregroundRadius2 = segmentationSettings.circumferenceTestSettings.foregroundRadius2;

            cv::Point probePosition = get_mouse_probe_position();

            std::vector<astra::Vector2i> points;

            segmentation::get_circumference_points(m_matDepth, probePosition, foregroundRadius1, mapper, points);

            for (auto p : points)
            {
                mark_image_pixel(imageFrame, color, p);
            }

            segmentation::get_circumference_points(m_matDepth, probePosition, foregroundRadius2, mapper, points);

            for (auto p : points)
            {
                mark_image_pixel(imageFrame, color, p);
            }

            cv::Point spawnPosition = get_spawn_position();
            RGBPixel spawnColor(255, 0, 255);

            mark_image_pixel(imageFrame, spawnColor, Vector2i(spawnPosition.x, spawnPosition.y));
        }

        void HandTracker::update_debug_image_frame(_astra_imageframe& colorFrame)
        {
            PROFILE_FUNC();
            float m_maxVelocity = 0.1;

            RGBPixel foregroundColor(0, 0, 255);
            RGBPixel searchedColor(128, 255, 0);
            RGBPixel searchedColor2(0, 128, 255);
            RGBPixel testPassColor(0, 255, 128);

            DebugHandViewType view = m_debugImageStream->view_type();

            switch (view)
            {
            case DEBUG_HAND_VIEW_DEPTH:
                m_debugVisualizer.showDepthMat(m_matDepth,
                                               colorFrame);
                break;
            case DEBUG_HAND_VIEW_DEPTH_MOD:
                m_debugVisualizer.showDepthMat(m_depthUtility.matDepthFilled(),
                                               colorFrame);
                break;
            case DEBUG_HAND_VIEW_DEPTH_AVG:
                m_debugVisualizer.showDepthMat(m_depthUtility.matDepthAvg(),
                                               colorFrame);
                break;
            case DEBUG_HAND_VIEW_VELOCITY:
                m_debugVisualizer.showVelocityMat(m_depthUtility.matDepthVel(),
                                                  m_maxVelocity,
                                                  colorFrame);
                break;
            case DEBUG_HAND_VIEW_FILTEREDVELOCITY:
                m_debugVisualizer.showVelocityMat(m_depthUtility.matDepthVelErode(),
                                                  m_maxVelocity,
                                                  colorFrame);
                break;
            case DEBUG_HAND_VIEW_UPDATE_SEGMENTATION:
                m_debugVisualizer.showNormArray<char>(m_debugUpdateSegmentation,
                                                      m_debugUpdateSegmentation,
                                                      colorFrame);
                break;
            case DEBUG_HAND_VIEW_CREATE_SEGMENTATION:
                            m_debugVisualizer.showNormArray<char>(m_debugCreateSegmentation,
                                                      m_debugCreateSegmentation,
                                                      colorFrame);
                break;
            case DEBUG_HAND_VIEW_UPDATE_SEARCHED:
            case DEBUG_HAND_VIEW_CREATE_SEARCHED:
                m_debugVisualizer.showDepthMat(m_matDepth,
                                               colorFrame);
                break;
            case DEBUG_HAND_VIEW_CREATE_SCORE:
                m_debugVisualizer.showNormArray<float>(m_debugCreateScore,
                                                       m_debugCreateSegmentation,
                                                       colorFrame);
                break;
            case DEBUG_HAND_VIEW_UPDATE_SCORE:
                m_debugVisualizer.showNormArray<float>(m_debugUpdateScore,
                                                       m_debugUpdateSegmentation,
                                                       colorFrame);
                break;
            case DEBUG_HAND_VIEW_HANDWINDOW:
                m_debugVisualizer.showDepthMat(m_matDepthWindow,
                                               colorFrame);
                break;
            case DEBUG_HAND_VIEW_TEST_PASS_MAP:
                m_debugVisualizer.showNormArray<char>(m_debugCreateTestPassMap,
                                                      m_debugCreateTestPassMap,
                                                      colorFrame);
                break;
            }

            if (view != DEBUG_HAND_VIEW_HANDWINDOW &&
                view != DEBUG_HAND_VIEW_CREATE_SCORE &&
                view != DEBUG_HAND_VIEW_UPDATE_SCORE &&
                view != DEBUG_HAND_VIEW_DEPTH_MOD &&
                view != DEBUG_HAND_VIEW_DEPTH_AVG &&
                view != DEBUG_HAND_VIEW_TEST_PASS_MAP)
            {
                if (view == DEBUG_HAND_VIEW_CREATE_SEARCHED)
                {
                    m_debugVisualizer.overlayMask(m_createForegroundSearched, colorFrame, searchedColor, PixelType::Searched);
                    m_debugVisualizer.overlayMask(m_createForegroundSearched, colorFrame, searchedColor2, PixelType::SearchedFromOutOfRange);
                }
                else if (view == DEBUG_HAND_VIEW_UPDATE_SEARCHED)
                {
                    m_debugVisualizer.overlayMask(m_updateForegroundSearched, colorFrame, searchedColor, PixelType::Searched);
                    m_debugVisualizer.overlayMask(m_updateForegroundSearched, colorFrame, searchedColor2, PixelType::SearchedFromOutOfRange);
                }

                m_debugVisualizer.overlayMask(m_matVelocitySignal, colorFrame, foregroundColor, PixelType::Foreground);
            }

            if (m_debugImageStream->use_mouse_probe())
            {
                overlay_circle(colorFrame);
            }
            m_debugVisualizer.overlayCrosshairs(m_pointProcessor.get_trackedPoints(), colorFrame);
        }
}}}
