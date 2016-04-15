#include "ScalingCoordinateMapper.h"
#include <opencv2/core/core.hpp>
#include <AstraUL/streams/Depth.h>
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

namespace astra { namespace plugins { namespace hand {

    void convert_depth_to_world_f(const conversion_cache_t& depthToWorldData,
                                  float depthX, float depthY, float depthZ,
                                  float& worldX, float& worldY, float& worldZ)
    {
        float normalizedX = depthX / depthToWorldData.resolutionX - .5f;
        float normalizedY = .5f - depthY / depthToWorldData.resolutionY;

        worldX = normalizedX * depthZ * depthToWorldData.xzFactor;
        worldY = normalizedY * depthZ * depthToWorldData.yzFactor;
        worldZ = depthZ;
    }

    cv::Point3f cv_convert_depth_to_world(const conversion_cache_t& depthToWorldData,
                                          float depthX, float depthY, float depthZ)
    {
        float worldX, worldY, worldZ;

        convert_depth_to_world_f(depthToWorldData,
                                 depthX, depthY, depthZ,
                                 worldX, worldY, worldZ);

        return cv::Point3f(worldX, worldY, worldZ);
    }

    cv::Point3f cv_convert_depth_to_world(const conversion_cache_t& depthToWorldData,
                                          int depthX, int depthY, float depthZ)
    {
        return cv_convert_depth_to_world(depthToWorldData,
                                         static_cast<float>(depthX),
                                         static_cast<float>(depthY),
                                         depthZ);
    }

    cv::Point3f cv_convert_depth_to_world(const conversion_cache_t& depthToWorldData,
                                          const cv::Point3f& depth)
    {
        return cv_convert_depth_to_world(depthToWorldData, depth.x, depth.y, depth.z);
    }

    cv::Point3f cv_convert_world_to_depth(const conversion_cache_t& depthToWorldData,
                                          float worldX, float worldY, float worldZ)
    {
        float depthX = depthToWorldData.coeffX * worldX / worldZ + depthToWorldData.halfResX;
        float depthY = depthToWorldData.halfResY - depthToWorldData.coeffY * worldY / worldZ;
        float depthZ = worldZ;

        return cv::Point3f(depthX, depthY, depthZ);
    }

    cv::Point3f cv_convert_world_to_depth(const conversion_cache_t& depthToWorldData,
                                          const cv::Point3f& world)
    {
        return cv_convert_world_to_depth(depthToWorldData, world.x, world.y, world.z);
    }


    cv::Point ScalingCoordinateMapper::offset_pixel_location_by_mm(const cv::Point& position,
                                                                   float offsetX,
                                                                   float offsetY,
                                                                   float depthZ) const
    {
        if (depthZ == 0)
        {
            return position;
        }

        const conversion_cache_t& depthToWorldData = m_depthToWorldData;

        const float scaledDepth = depthZ * m_scale;

        //This bakes the world to depth conversion and full size to scaled conversion into
        //a single factor that we apply to the requested offset.
        //This process is equivalent to depth_to_world, adding offsetX/Y to world position,
        //then doing world_to_depth.

        const float xFactor = depthToWorldData.resolutionX / (scaledDepth * depthToWorldData.xzFactor);
        const float finalDepthX = position.x + offsetX * xFactor;

        const float yFactor = depthToWorldData.resolutionY / (scaledDepth * depthToWorldData.yzFactor);
        const float finalDepthY = position.y - offsetY * yFactor;

        return cv::Point(static_cast<int>(finalDepthX), static_cast<int>(finalDepthY));
    }
}}}
