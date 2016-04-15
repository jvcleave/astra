#ifndef HANDSTREAM_H
#define HANDSTREAM_H

#include <Astra/Plugins/SingleBinStream.h>
#include <AstraUL/streams/hand_types.h>
#include <AstraUL/astraul_ctypes.h>
#include <AstraUL/Plugins/stream_types.h>
#include <string.h>
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

namespace astra { namespace plugins { namespace hand {

    class HandStream : public SingleBinStream<astra_handframe_wrapper_t>
    {
    public:
        HandStream(PluginServiceProxy& pluginService,
                   astra_streamset_t streamSet,
                   size_t maxHandCount)
            : SingleBinStream(pluginService,
                              streamSet,
                              StreamDescription(ASTRA_STREAM_HAND,
                                                DEFAULT_SUBTYPE),
                              sizeof(astra_handpoint_t) * maxHandCount)
        { }

        bool include_candidate_points() const { return m_includeCandidatePoints; }
        void set_include_candidate_points(bool includeCandidatePoints)
        {
            m_includeCandidatePoints = includeCandidatePoints;
        }
    protected:
        virtual void on_set_parameter(astra_streamconnection_t connection,
                                      astra_parameter_id id,
                                      size_t inByteLength,
                                      astra_parameter_data_t inData) override;

        virtual void on_get_parameter(astra_streamconnection_t connection,
                                      astra_parameter_id id,
                                      astra_parameter_bin_t& parameterBin) override;
    private:
        void get_include_candidates(astra_parameter_bin_t& parameterBin);
        void set_include_candidates(size_t inByteLength, astra_parameter_data_t& inData);

        virtual void on_connection_removed(astra_bin_t bin,
                                           astra_streamconnection_t connection) override
        {
            SingleBinStream::on_connection_removed(bin, connection);

            #ifdef __ANDROID__
                PROFILE_UPDATE();
                PROFILE_OUTPUT("/sdcard/profile_orbbec_hand.txt");
            #endif
        }

        bool m_includeCandidatePoints{ false };
    };

}}}

#endif /* HANDSTREAM_H */
