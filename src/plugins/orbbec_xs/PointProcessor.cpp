#include "PointProcessor.h"
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

namespace astra { namespace plugins { namespace xs {

    PointProcessor::PointProcessor(PluginServiceProxy& pluginService,
                                   astra_streamset_t streamset,
                                   StreamDescription& depthDesc)
        : m_streamset(get_uri_for_streamset(pluginService, streamset)),
          m_streamSet(streamset),
          m_reader(m_streamset.create_reader()),
          m_depthStream(m_reader.stream<DepthStream>(depthDesc.subtype())),
          m_pluginService(pluginService)
    {
        m_depthStream.start();
        m_reader.addListener(*this);
    }

    PointProcessor::~PointProcessor()
    {
    }

    void PointProcessor::on_frame_ready(StreamReader& reader, Frame& frame)
    {
        DepthFrame depthFrame = frame.get<DepthFrame>();

        create_point_stream_if_necessary(depthFrame);

        if (m_pointStream->has_connections())
        {
            LOG_TRACE("PointProcessor", "updating point frame");
            update_pointframe_from_depth(depthFrame);
        }
    }

    void PointProcessor::create_point_stream_if_necessary(DepthFrame& depthFrame)
    {
        if (m_pointStream != nullptr)
        {
            return;
        }

        //TODO check for changes in depthFrame width and height and update bin size
        LOG_INFO("PointProcessor", "creating point stream");

        int width = depthFrame.resolutionX();
        int height = depthFrame.resolutionY();

        auto ps = make_stream<PointStream>(m_pluginService, m_streamSet, width, height);
        m_pointStream = std::unique_ptr<PointStream>(std::move(ps));

        LOG_INFO("PointProcessor", "created point stream");

        m_depthConversionCache = m_depthStream.depth_to_world_data();
    }

    void PointProcessor::update_pointframe_from_depth(DepthFrame& depthFrame)
    {
        //use same frameIndex as source depth frame
        astra_frame_index_t frameIndex = depthFrame.frameIndex();

        astra_imageframe_wrapper_t* pointFrameWrapper = m_pointStream->begin_write(frameIndex);

        if (pointFrameWrapper != nullptr)
        {
            pointFrameWrapper->frame.frame = nullptr;
            pointFrameWrapper->frame.data = &pointFrameWrapper->frame_data[0];

            astra_image_metadata_t metadata;

            metadata.width = depthFrame.resolutionX();
            metadata.height = depthFrame.resolutionY();
            metadata.pixelFormat = ASTRA_PIXEL_FORMAT_POINT;

            pointFrameWrapper->frame.metadata = metadata;

            Vector3f* p_points = reinterpret_cast<Vector3f*>(pointFrameWrapper->frame.data);
            calculate_point_frame(depthFrame, p_points);

            m_pointStream->end_write();
        }
    }

    void PointProcessor::calculate_point_frame(DepthFrame& depthFrame,
                                               Vector3f* p_points)
    {
        int width = depthFrame.resolutionX();
        int height = depthFrame.resolutionY();
        const int16_t* p_depth = depthFrame.data();

        const conversion_cache_t conversionData = m_depthConversionCache;

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x, ++p_points, ++p_depth)
            {
                uint16_t depth = *p_depth;
                Vector3f& point = *p_points;

                float normalizedX = static_cast<float>(x) / conversionData.resolutionX - .5f;
                float normalizedY = .5f - static_cast<float>(y) / conversionData.resolutionY;

                point.x = normalizedX * depth * conversionData.xzFactor;
                point.y = normalizedY * depth * conversionData.yzFactor;
                point.z = depth;
            }
        }
    }

}}}
