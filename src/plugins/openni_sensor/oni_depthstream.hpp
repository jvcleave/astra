#ifndef ONI_DEPTHSTREAM_H
#define ONI_DEPTHSTREAM_H

#include "oni_devicestream.hpp"
#include <Astra/Plugins/plugin_capi.h>
#include <AstraUL/Plugins/stream_types.h>
#include <AstraUL/astraul_ctypes.h>
#include <cmath>
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

namespace orbbec { namespace ni {

    class depthstream : public devicestream<astra_imageframe_wrapper_t>
    {
    public:
        depthstream(astra::PluginServiceProxy& pluginService,
                    astra_streamset_t streamSet,
                    openni::Device& oniDevice,
                    stream_listener& listener);

        depthstream(const depthstream&) = delete;
        depthstream& operator=(const depthstream&) = delete;

    private:
        conversion_cache_t conversionCache_;

        void refresh_conversion_cache(float horizontalFov,
                                      float verticalFov,
                                      int resolutionX,
                                      int resolutionY);

        virtual astra_status_t on_open() override;

        virtual void on_get_parameter(astra_streamconnection_t connection,
                                      astra_parameter_id id,
                                      astra_parameter_bin_t& parameterBin) override;

        void on_set_parameter(astra_streamconnection_t connection,
                              astra_parameter_id id,
                              size_t inByteLength,
                              astra_parameter_data_t inData) override;

        virtual void on_connection_removed(astra_bin_t bin,
                                           astra_streamconnection_t connection) override;
    };
}}

#endif /* ONIDEPTHSTREAM_H */
