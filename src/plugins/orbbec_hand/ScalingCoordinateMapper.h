#ifndef SCALINGCOORDINATEMAPPER_H
#define SCALINGCOORDINATEMAPPER_H

#include <opencv2/core/core.hpp>
#include <AstraUL/streams/Depth.h>
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

namespace astra { namespace plugins { namespace hand {

    class ScalingCoordinateMapper;

    void convert_depth_to_world_f(const conversion_cache_t& depthToWorldData,
                                  float depthX, float depthY, float depthZ,
                                  float& worldX, float& worldY, float& worldZ);

    cv::Point3f cv_convert_depth_to_world(const conversion_cache_t& depthToWorldData,
                                          int depthX, int depthY, float depthZ);

    cv::Point3f cv_convert_depth_to_world(const conversion_cache_t& depthToWorldData,
                                          float depthX, float depthY, float depthZ);

    cv::Point3f cv_convert_depth_to_world(const conversion_cache_t& depthToWorldData,
                                          const cv::Point3f& depth);

    cv::Point3f cv_convert_world_to_depth(const conversion_cache_t& depthToWorldData,
                                          float worldX, float worldY, float worldZ);

    cv::Point3f cv_convert_world_to_depth(const conversion_cache_t& depthToWorldData,
                                          const cv::Point3f& world);

    class ScalingCoordinateMapper
    {
    public:
        ScalingCoordinateMapper(const conversion_cache_t depthToWorldData,
                                const float scale,
                                const float offsetX = 0,
                                const float offsetY = 0)
            : m_depthToWorldData(depthToWorldData),
              m_scale(scale),
              m_offsetX(offsetX),
              m_offsetY(offsetY)
        { }

        inline cv::Point3f convert_depth_to_world(int depthX, int depthY, float depthZ) const
        {
            PROFILE_FUNC();
            depthX = static_cast<int>((depthX + m_offsetX) * m_scale);
            depthY = static_cast<int>((depthY + m_offsetY) * m_scale);

            return cv_convert_depth_to_world(m_depthToWorldData, depthX, depthY, depthZ);
        }

        inline void convert_depth_to_world(int depthX, int depthY, float depthZ,
                                           float& worldX, float& worldY, float& worldZ) const
        {
            PROFILE_FUNC();
            depthX = static_cast<int>((depthX + m_offsetX) * m_scale);
            depthY = static_cast<int>((depthY + m_offsetY) * m_scale);

            convert_depth_to_world_f(m_depthToWorldData,
                                     depthX, depthY, static_cast<int>(depthZ),
                                     worldX, worldY, worldZ);
        }

        inline cv::Point3f convert_depth_to_world(float depthX, float depthY, float depthZ) const
        {
            PROFILE_FUNC();
            depthX = (depthX + m_offsetX) * m_scale;
            depthY = (depthY + m_offsetY) * m_scale;

            return cv_convert_depth_to_world(m_depthToWorldData, depthX, depthY, depthZ);
        }

        inline cv::Point3f convert_depth_to_world(cv::Point3f depthPosition) const
        {
            PROFILE_FUNC();
            return convert_depth_to_world(depthPosition.x, depthPosition.y, depthPosition.z);
        }

        inline cv::Point3f convert_world_to_depth(cv::Point3f worldPosition) const
        {
            PROFILE_FUNC();
            cv::Point3f depth = cv_convert_world_to_depth(m_depthToWorldData, worldPosition);

            depth.x = (depth.x / m_scale) - m_offsetX;
            depth.y = (depth.y / m_scale) - m_offsetY;

            return depth;
        }

        cv::Point offset_pixel_location_by_mm(const cv::Point& position,
                                              float offsetX,
                                              float offsetY,
                                              float depthZ) const;

        inline float scale() const { return m_scale; }
        inline float offsetX() const { return m_offsetX; }
        inline float offsetY() const { return m_offsetY; }

    private:
        const conversion_cache_t m_depthToWorldData;
        const float m_scale;
        const float m_offsetX;
        const float m_offsetY;
    };

}}}

#endif // SCALINGCOORDINATEMAPPER_H
