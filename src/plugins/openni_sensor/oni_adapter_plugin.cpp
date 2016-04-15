#include <AstraUL/astraul_ctypes.h>
#define PROFILE_FUNC(...) 
#define PROFILE_BEGIN(...) 
#define PROFILE_END(...) 
#define PROFILE_UPDATE(...) 
#define PROFILE_OUTPUT(...) 

#include <cstring>
#include <sstream>

#include "oni_adapter_plugin.hpp"
#include "oni_depthstream.hpp"
#include "oni_colorstream.hpp"

EXPORT_PLUGIN(orbbec::ni::oni_adapter_plugin)

namespace orbbec { namespace ni {

        void oni_adapter_plugin::init_openni()
        {
            PROFILE_FUNC();
            openni::Version version = openni::OpenNI::getVersion();

            LOG_INFO("orbbec.ni.oni_adapter_plugin", "Initializing OpenNI v%d.%d.%d.%d",
                  version.major,
                  version.minor,
                  version.maintenance,
                  version.build);

            openni::Status rc = openni::STATUS_OK;

            openni::OpenNI::addDeviceConnectedListener(this);
            openni::OpenNI::addDeviceDisconnectedListener(this);

            rc = openni::OpenNI::initialize();

            bool successful = rc == openni::STATUS_OK;

            if (!successful)
            {
                LOG_WARN("orbbec.ni.oni_adapter_plugin", "Failed to initialize OpenNI: %s", openni::OpenNI::getExtendedError());
            }
            else
            {
                LOG_INFO("orbbec.ni.oni_adapter_plugin", "Initialized OpenNI v%d.%d.%d.%d",
                      version.major,
                      version.minor,
                      version.maintenance,
                      version.build);
            }
        }

        void oni_adapter_plugin::on_host_event(astra_event_id id, const void* data, size_t dataSize)
        {
            PROFILE_FUNC();
#ifdef __ANDROID__
            switch (id)
            {
            case ASTRA_EVENT_RESOURCE_AVAILABLE:
                const char* resourceUri = static_cast<const char*>(data);

                LOG_INFO("orbbec.ni.oni_adapter_plugin", "resource uri received: %s", resourceUri);

                unsigned int vid = 0;
                unsigned int pid = 0;
                unsigned int bus = 0;
                unsigned int address = 0;

                int scanned = sscanf(resourceUri, "usb/%u/%u/%u/%u", &vid, &pid, &bus, &address);

                if (scanned == 4)
                {
                    char oniUri[1024];
                    snprintf(oniUri, 1024, "%04hx/%04hx@%hhu/%hhu", vid, pid, bus, address);
                    LOG_INFO("orbbec.ni.oni_adapter_plugin", "parsed oniUri: %s", oniUri);

                    openni::Array<openni::DeviceInfo> devices;
                    openni::OpenNI::enumerateDevices(&devices);
                    LOG_INFO("orbbec.ni.oni_adapter_plugin", "num devices: %d", devices.getSize());

                    for(int i = 0; i < devices.getSize(); i++)
                    {
                        const openni::DeviceInfo& info = devices[i];
                        LOG_INFO("orbbec.ni.oni_adapter_plugin", "found sensor: %s", info.getUri());
                        if (std::strcmp(oniUri, info.getUri()) == 0)
                        {
                            LOG_INFO("orbbec.ni.oni_adapter_plugin", "device connected: %s", info.getUri());
                            add_or_get_device(info.getUri());
                            break;
                        }
                    }
                }
                else
                {
                    LOG_INFO("orbbec.ni.oni_adapter_plugin", "unknown resource uri: %s", resourceUri);
                }
            }
#endif
        }

        device_streamset* oni_adapter_plugin::add_or_get_device(const char* oniUri)

        {
            device_streamset* device = find_device(oniUri);

            if (device)
                return device;

            std::stringstream sstream;
            sstream << "device/sensor" << streamsets_.size();

            streamset_ptr streamSet = std::make_unique<device_streamset>(sstream.str(),
                                                                         pluginService(),
                                                                         oniUri);
            streamSet->open();
            device = streamSet.get();
            streamsets_.push_back(std::move(streamSet));

            return device;
        }

        device_streamset* oni_adapter_plugin::find_device(const char* oniUri)
        {
            auto it = std::find_if(streamsets_.begin(), streamsets_.end(),
                                   [&oniUri] (streamset_ptr& setPtr) -> bool
                                   {
                                       return setPtr->get_uri() == oniUri;
                                   });

            return it != streamsets_.end() ? it->get() : nullptr;
        }

        void oni_adapter_plugin::onDeviceConnected(const openni::DeviceInfo* info)
        {
            PROFILE_FUNC();
#ifndef __ANDROID__
            LOG_INFO("orbbec.ni.oni_adapter_plugin", "device connected: %s", info->getUri());
            add_or_get_device(info->getUri());
#endif
        }

        void oni_adapter_plugin::onDeviceDisconnected(const openni::DeviceInfo* info)
        {
            PROFILE_FUNC();
            LOG_INFO("orbbec.ni.oni_adapter_plugin", "device disconnected: %s", info->getUri());
            auto it = std::find_if(streamsets_.begin(), streamsets_.end(),
                                   [&info] (streamset_ptr& setPtr)
                                   -> bool
                                   {
                                       return setPtr->get_uri() == info->getUri();
                                   });

            streamsets_.erase(it);
        }

        oni_adapter_plugin::~oni_adapter_plugin()
        {
            PROFILE_FUNC();
#ifndef __ANDROID__
            PROFILE_UPDATE();
            PROFILE_OUTPUT("profile_openni_sensor.txt");
#endif

            streamsets_.clear();
            LOG_INFO("orbbec.ni.oni_adapter_plugin", "shutting down openni");
            openni::OpenNI::shutdown();
        }

        void oni_adapter_plugin::temp_update()
        {
            PROFILE_FUNC();
            read_streams();
            PROFILE_UPDATE();
        }

        astra_status_t oni_adapter_plugin::read_streams()
        {
            PROFILE_FUNC();
            for(auto& set : streamsets_)
            {
                set->read();
            }

            return ASTRA_STATUS_SUCCESS;
        }
    }}
