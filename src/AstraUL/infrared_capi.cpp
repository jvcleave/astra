#include <memory.h>
#include <cstring>
#include <unordered_map>

#include <Astra/astra_types.h>
#include <AstraUL/streams/infrared_types.h>
#include <AstraUL/Plugins/stream_types.h>
#include <AstraUL/streams/image_capi.h>
#include <AstraUL/streams/image_parameters.h>
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

#include "generic_stream_api.h"

ASTRA_BEGIN_DECLS

ASTRA_API_EX astra_status_t astra_reader_get_infraredstream(astra_reader_t reader,
                                                            astra_infraredstream_t* infraredStream)

{
    return astra_reader_get_stream(reader,
                                   ASTRA_STREAM_INFRARED,
                                   DEFAULT_SUBTYPE,
                                   infraredStream);
}

ASTRA_API_EX astra_status_t astra_infraredstream_get_hfov(astra_infraredstream_t infraredStream,
                                                          float* hFov)
{
    return astra_stream_get_parameter_fixed(infraredStream,
                                            ASTRA_PARAMETER_IMAGE_HFOV,
                                            sizeof(float),
                                            reinterpret_cast<astra_parameter_data_t*>(hFov));
}

ASTRA_API_EX astra_status_t astra_infraredstream_get_vfov(astra_infraredstream_t infraredStream,
                                                          float* vFov)
{
    return astra_stream_get_parameter_fixed(infraredStream,
                                            ASTRA_PARAMETER_IMAGE_VFOV,
                                            sizeof(float),
                                            reinterpret_cast<astra_parameter_data_t*>(vFov));
}

ASTRA_API_EX astra_status_t astra_frame_get_infraredframe(astra_reader_frame_t readerFrame,
                                                          astra_infraredframe_t* infraredFrame)
{
    return astra_reader_get_imageframe(readerFrame,
                                       ASTRA_STREAM_INFRARED,
                                       DEFAULT_SUBTYPE,
                                       infraredFrame);
}

ASTRA_API_EX astra_status_t astra_frame_get_infraredframe_with_subtype(astra_reader_frame_t readerFrame,
                                                                       astra_stream_subtype_t subtype,
                                                                       astra_infraredframe_t* infraredFrame)
{
    return astra_reader_get_imageframe(readerFrame,
                                       ASTRA_STREAM_INFRARED,
                                       subtype,
                                       infraredFrame);
}

ASTRA_API_EX astra_status_t astra_infraredframe_get_frameindex(astra_infraredframe_t infraredFrame,
                                                               astra_frame_index_t* index)
{
    return astra_generic_frame_get_frameindex(infraredFrame, index);
}

ASTRA_API_EX astra_status_t astra_infraredframe_get_data_byte_length(astra_infraredframe_t infraredFrame,
                                                                     size_t* byteLength)
{
    return astra_imageframe_get_data_byte_length(infraredFrame, byteLength);
}

ASTRA_API_EX astra_status_t astra_infraredframe_get_data_ptr(astra_infraredframe_t infraredFrame,
                                                             uint8_t** data,
                                                             size_t* byteLength)
{
    void* voidData = nullptr;
    astra_imageframe_get_data_ptr(infraredFrame, &voidData, byteLength);
    *data = static_cast<uint8_t*>(voidData);

    return ASTRA_STATUS_SUCCESS;
}

ASTRA_API_EX astra_status_t astra_infraredframe_copy_data(astra_infraredframe_t infraredFrame,
                                                          uint8_t* data)
{
    return astra_imageframe_copy_data(infraredFrame, data);
}

ASTRA_API_EX astra_status_t astra_infraredframe_get_metadata(astra_infraredframe_t infraredFrame,
                                                             astra_image_metadata_t* metadata)
{
    return astra_imageframe_get_metadata(infraredFrame, metadata);
}

ASTRA_END_DECLS
