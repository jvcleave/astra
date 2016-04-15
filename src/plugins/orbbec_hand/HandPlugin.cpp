#include "HandSettings.h"
#include "HandPlugin.h"
#include "HandTracker.h"
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 

EXPORT_PLUGIN(astra::plugins::hand::HandPlugin);

namespace astra { namespace plugins { namespace hand {

    const char HANDPLUGIN_CONFIG_FILE[] = "plugins/orbbec_hand.toml";

    HandPlugin::HandPlugin(PluginServiceProxy* pluginProxy)
        : PluginBase(pluginProxy, "orbbec_hand")
    {
        m_settings = parse_settings(HANDPLUGIN_CONFIG_FILE);
    }

    HandPlugin::~HandPlugin()
    {
#ifndef __ANDROID__
        PROFILE_UPDATE();
        PROFILE_OUTPUT("profile_orbbec_hand.txt");
#endif

        pluginService().unregister_stream_registered_callback(m_streamAddedCallbackId);
        pluginService().unregister_stream_unregistering_callback(m_streamRemovingCallbackId);
    }

    void HandPlugin::on_initialize()
    {
        pluginService().register_stream_registered_callback(&HandPlugin::stream_registered_handler_thunk,
                                                            this,
                                                            &m_streamAddedCallbackId);

        pluginService().register_stream_unregistering_callback(&HandPlugin::stream_unregistering_handler_thunk,
                                                               this,
                                                               &m_streamRemovingCallbackId);
    }

    void HandPlugin::stream_registered_handler_thunk(void* clientTag,
                                                     astra_streamset_t setHandle,
                                                     astra_stream_t streamHandle,
                                                     astra_stream_desc_t desc)
    {
        HandPlugin* plugin = static_cast<HandPlugin*>(clientTag);
        plugin->stream_registered_handler(setHandle, streamHandle, desc);
    }

    void HandPlugin::stream_unregistering_handler_thunk(void* clientTag,
                                                        astra_streamset_t setHandle,
                                                        astra_stream_t streamHandle,
                                                        astra_stream_desc_t desc)

    {
        HandPlugin* plugin = static_cast<HandPlugin*>(clientTag);
        plugin->stream_unregistering_handler(setHandle, streamHandle, desc);
    }

    void HandPlugin::stream_registered_handler(astra_streamset_t setHandle,
                                               astra_stream_t streamHandle,
                                               astra_stream_desc_t streamDesc)
    {
        if (streamDesc.type == ASTRA_STREAM_DEPTH &&
            m_streamTrackerMap.find(streamHandle) == m_streamTrackerMap.end())
        {
            StreamDescription depthDescription = streamDesc;

            HandTracker* tracker = new HandTracker(pluginService(),
                                                   setHandle,
                                                   depthDescription,
                                                   m_settings);

            m_streamTrackerMap[streamHandle] = tracker;
        }
    }

    void HandPlugin::stream_unregistering_handler(astra_streamset_t setHandle,
                                                  astra_stream_t streamHandle,
                                                  astra_stream_desc_t desc)
    {
        auto it = m_streamTrackerMap.find(streamHandle);
        if (it != m_streamTrackerMap.end())
        {
            HandTracker* tracker = it->second;

            delete tracker;
            m_streamTrackerMap.erase(it);
        }
    }
}}}
