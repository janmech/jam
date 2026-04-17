#pragma once

#include <map>
#include "helios-sdk/cpp/HeliosDac.h"
#include "helios-sdk/cpp/libusb.h"
#include "c74_min_api.h"
#include "shared/InterfaceHeliosListener.hpp"

using number = c74::min::number;

namespace jam::helios {
    
    enum DacType {
        UNKNOWN = 0,
        USB = 1,
        IDN = 2,
    };
    
    enum DeviceState {
        ATTACH_ERROR_ALREADY_ATTACHED = -2,
        NOTFOUND = -1,
        ATTACH_SUCCESS = 0,
    };
    
    typedef struct DeviceInfo {
        std::string name = "unknown";
        int firmware = 0;
        DacType type = DacType::UNKNOWN;
        int index = -1;
        c74::max::t_object *owner = NULL;
    } device_info_t;
    
    class Connector {
        
    public:
        Connector(const Connector&) = delete;
        
        ~Connector() {
            delete _opened_dac_devices;
        }
        
        static Connector & get() {
            static Connector instance;
            return instance;
        }
        
        int deviceScan();
        
        bool isScanning();
        
        void registerJamHeliosInstance(InterfaceHeliosListener * ptr_jam_helio_instance);
        void unRegisterJamHeliosInstance(InterfaceHeliosListener * ptr_jam_helio_instance);
        void _callListeners();
        
        
        std::vector<device_info_t> * getOpenDevices();
        
        std::string getTypeName(DacType type);
        
        bool getDacDeviceByIndex(int index, device_info_t * device_info);
        
        DeviceState attachDacDevice(int device_index, uint instance_id);
        
        void detachDacDevice(uint instance_id);
        
        HeliosDac * getDac();
        
          
    protected:
        std::atomic<bool> _is_scanning = false;
        
        HeliosDac _helios_dac;
        
        
        std::mutex _opened_dac_devices_lock;
        std::vector<device_info_t> *_opened_dac_devices;
        
        std::mutex _attached_dac_devices_lock;
        std::map<int, uint> _attached_dac_devices = {};
        
        std::vector<InterfaceHeliosListener*> _jam_helios_instances;
        std::mutex _jam_helios_instances_lock;
        
        
        
    private:
        Connector() {
            _opened_dac_devices = new std::vector<device_info_t>();
        };
    };
}
