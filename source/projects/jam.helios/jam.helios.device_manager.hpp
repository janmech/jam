    //
    //  jam.helios.hpp
    //  jam.helios
    //
    //  Created by Jan Mech on 12/3/25.
    //

#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include "helios-sdk/cpp/HeliosDac.h"
#include "helios-sdk/cpp/libusb.h"
#include "c74_min_api.h"

namespace jam::helios {
    enum DacType {
        UNKNOWN = 0,
        USB = 1,
        IDN = 2,
    };
    
    typedef struct DeviceInfo {
        std::string name = "unknown";
        int firmware = 0;
        std::string type = "";
        int index = -1;
        
    } device_info_t;
    
    class DeviceManager{
        
    public:
        std::vector<device_info_t> & getOpenDevices();
        
        DeviceManager(const DeviceManager&) = delete;
        ~DeviceManager() {
            std::cout << "DeviceManager Desctructed" << std::endl;
        }
        
        static DeviceManager & get() {
            static DeviceManager instance;
            return instance;
        }
        
        int deviceScan();
        bool isScanning();
        void addObjInstance(c74::max::t_object *intstance);
        void removeObjInstance(c74::max::t_object *intstance);
        
        
    protected:
        bool _is_scanning = false;
        HeliosDac _helios_dac;
        std::vector<device_info_t> _open_devices;
        std::vector<c74::max::t_object*>_object_instances;
        
        
    private:
        DeviceManager() {}
    };
    
}


