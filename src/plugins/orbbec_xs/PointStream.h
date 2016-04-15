#ifndef POINTSTREAM_H
#define POINTSTREAM_H

#include <Astra/Plugins/SingleBinStream.h>
#include <AstraUL/streams/point_types.h>
#include <AstraUL/astraul_ctypes.h>
#include <AstraUL/Plugins/stream_types.h>
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

namespace astra { namespace plugins { namespace xs {

    class PointStream : public astra::plugins::SingleBinStream<astra_imageframe_wrapper_t>
    {
    public:
        PointStream(PluginServiceProxy& pluginService,
                    astra_streamset_t streamSet,
                    uint32_t width,
                    uint32_t height)
            : SingleBinStream(pluginService,
                              streamSet,
                              StreamDescription(ASTRA_STREAM_POINT,
                                                DEFAULT_SUBTYPE),
                              width * height * sizeof(astra_vector3f_t))
        {}
    };
}}}

#endif /* POINTSTREAM_H */
