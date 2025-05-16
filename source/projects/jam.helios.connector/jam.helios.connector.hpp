#pragma once

#include <map>
#include "helios-sdk/cpp/HeliosDac.h"
#include "helios-sdk/cpp/libusb.h"
#include "c74_min_api.h"

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
            delete _open_devices;
        }
        
        static Connector & get() {
            static Connector instance;
            return instance;
        }
        
        int deviceScan();
        
        bool isScanning();
        
        std::vector<device_info_t> * getOpenDevices();
        
        std::string getTypeName(DacType type);
        
        bool getDeviceByIndex(int index, device_info_t * device_info);
        
        DeviceState attachDevice(int device_index, uint instance_id);
        
        void detachDevice(uint instance_id);
        
        HeliosDac * getDac();
        
        
        
        
    protected:
        bool _is_scanning = false;
        
        HeliosDac _helios_dac;
        
        std::mutex _open_dev_lock;
        
        std::vector<device_info_t> *_open_devices;
        
        std::map<int, uint> _attached_devices = {};
        
    private:
        Connector() {
            _open_devices = new std::vector<device_info_t>();
        };
    };
}
