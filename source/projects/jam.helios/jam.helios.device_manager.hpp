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
#include <algorithm>
#include <mutex>
#include "helios-sdk/cpp/HeliosDac.h"
#include "helios-sdk/cpp/libusb.h"
#include "c74_min_api.h"

namespace jam::helios {
    enum DacType {
        UNKNOWN = 0,
        USB = 1,
        IDN = 2,
    };
    
    enum DeviceState {
        ATTACHED_ERROR_ALREADY_ATTACHED = -2,
        NOTFOUND = -1,
        ATTACHED_SUCCESS = 0,
    };
    
    typedef struct DeviceInfo {
        std::string name = "unknown";
        int firmware = 0;
        DacType type = DacType::UNKNOWN;
        int index = -1;
        c74::max::t_object *owner = NULL;
    } device_info_t;
    
    
    
    class DeviceManager{
        
    public:
        DeviceManager(const DeviceManager&) = delete;
        ~DeviceManager() {
            delete _open_devices;
        }
        
        static DeviceManager & get() {
            static DeviceManager instance;
            return instance;
        }
        
        std::vector<device_info_t>  *getOpenDevices();
        int deviceScan();
        bool isScanning();
        void addObjInstance(c74::max::t_object *intstance);
        void removeObjInstance(c74::max::t_object *intstance);
        void notifyInstances();
        std::string getTypeName(DacType type);
        DeviceState attachDeviceToInstance(int device_index, c74::max::t_object *intstance);
        DeviceState attachDeviceToInstance(std::string device_name, c74::max::t_object *intstance);
        bool detachDeviceFromInstance(c74::max::t_object *intstance);
        void setShutter( c74::max::t_object *intstance, bool state);
        void setTest(std::string s) {
            this->_test = s;
        };
        
        std::string getTest() {
            return this->_test;
        }
        
        
    protected:
        std::string _test = "";
        std::mutex _open_dev_lock;
        bool _is_scanning = false;
        HeliosDac _helios_dac;
        std::vector<device_info_t> *_open_devices;
        std::vector<c74::max::t_object*>_object_instances;
        
        device_info_t*  _findDevice(std::string name);
        device_info_t* _findDevice(int index);
        DeviceState _attachDevice(device_info_t *device, c74::max::t_object *intstance);
        
        
    private:
        DeviceManager() {
            _open_devices = new std::vector<device_info_t>();
        }
    };
    
}


