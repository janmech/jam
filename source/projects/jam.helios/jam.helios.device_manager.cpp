    //
    //  jam.helios.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 12/3/25.
    //

#include "./jam.helios.device_manager.hpp"

namespace jam::helios {
    
    int DeviceManager::deviceScan() {
        this->_is_scanning = true;
        this->_open_devices.clear();
        int numDevs = this->_helios_dac.OpenDevices();
        if (numDevs > 0) {
                //            cout << "Found" << numDevs << "DAC(s)" << endl;
            for (int j = 0; j < numDevs; j++) {
                device_info_t dac;
                dac.index = j;
                char name[32] = {0};
                if (this->_helios_dac.GetName(j, name) == HELIOS_SUCCESS) {
                    dac.name = std::string(name);
                }
                
                int firmware = this->_helios_dac.GetFirmwareVersion(j);
                
                if (firmware >= 0) {
                    dac.firmware = firmware;
                }
                
                int is_usb = this->_helios_dac.GetIsUsb(j);
                
                if(is_usb < 0) {
                    dac.type = DacType::UNKNOWN;
                } else if (is_usb == 1){
                    dac.type = DacType::USB;
                } else {
                    dac.type = DacType::IDN;
                }
                
                this->_open_devices.push_back(dac);
            }
        }
        this->_is_scanning = false;
        return numDevs;
    }
    
    std::vector<device_info_t> & DeviceManager::getOpenDevices() {
        return this->_open_devices;;
    };
    
    bool DeviceManager::isScanning() {
        return this->_is_scanning;
    }
    
    void DeviceManager::addObjInstance(c74::max::t_object *intstance) {
        this->_object_instances.push_back(intstance);
    };
    
    void DeviceManager::removeObjInstance(c74::max::t_object *intstance) {
        
        bool test = false;
//        this->_object_instances.push_back(intstance);
    };
    
}
