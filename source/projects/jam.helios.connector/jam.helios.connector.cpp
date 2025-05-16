#include "jam.helios.connector.hpp"

namespace jam::helios {
    
    bool Connector::isScanning() {
        return this->_is_scanning;
    }
    
    std::vector<device_info_t> *Connector::getOpenDevices() {
        this->_open_dev_lock.lock();
        std::vector<device_info_t> *open_devs = this->_open_devices;
        this->_open_dev_lock.unlock();
        return open_devs;
    };
    
    std::string Connector::getTypeName(DacType type) {
        switch (type) {
            case DacType::USB :
                return "USB";
            case DacType::IDN :
                return "IDN";
            case DacType::UNKNOWN :
            default:
                return "UNKNOWN";
        }
    };
    
    int Connector::deviceScan() {
        this->_is_scanning = true;
        this->_helios_dac.CloseDevices();
        
        this->_open_dev_lock.lock();
        this->_open_devices->clear();
        this->_attached_devices.clear();
        this->_open_dev_lock.unlock();
        
        int numDevs = this->_helios_dac.OpenDevices();
        if (numDevs > 0) {
            for (int i = 0; i < numDevs; i++) {
                device_info_t dac;
                dac.index = i;
                char name[32] = {0};
                if (this->_helios_dac.GetName(i, name) == HELIOS_SUCCESS) {
                    dac.name = std::string(name);
                }
                
                int firmware = this->_helios_dac.GetFirmwareVersion(i);
                
                if (firmware >= 0) {
                    dac.firmware = firmware;
                }
                
                int is_usb = this->_helios_dac.GetIsUsb(i);
                
                if(is_usb < 0) {
                    dac.type = DacType::UNKNOWN;
                } else if (is_usb == 1){
                    dac.type = DacType::USB;
                } else {
                    dac.type = DacType::IDN;
                }
                
                dac.index = i;
                
                this->_open_dev_lock.lock();
                this->_open_devices->push_back(dac);
                this->_open_dev_lock.unlock();
            }
        }
        this->_is_scanning = false;
        return numDevs;
    }
    
    DeviceState Connector::attachDevice(int device_index, uint instance_id) {
        // check if device withn index exists
        bool device_exists = false;
        for( auto it = this->_open_devices->begin(); it != this->_open_devices->end(); ++it) {
            if (it->index == device_index) {
                device_exists = true;
                break;
            }
        }
        if(!device_exists) {
            return DeviceState::NOTFOUND;
        }
        
        auto it = this->_attached_devices.find(device_index);
        if(it != this->_attached_devices.end()) {
            // device is already attached to this instace. Return success
            if(it->second == instance_id) {
                return DeviceState::ATTACH_SUCCESS ;
            } else {
                return DeviceState::ATTACH_ERROR_ALREADY_ATTACHED;
            }
        }
        
        // device exist and is not yet attached: let's attach it
        this->_open_dev_lock.lock();
        this->_attached_devices.insert({device_index, instance_id});
        this->_open_dev_lock.unlock();
        return DeviceState::ATTACH_SUCCESS ;
    };
    
    void Connector::detachDevice(uint instance_id) {
        for (const auto& pair : this->_attached_devices) {
            if(pair.second == instance_id) {
                this->_attached_devices.erase(pair.first);
                break;
                
            }
        }
    };
    
    HeliosDac * Connector::getDac() {
        return &this->_helios_dac;
    }
};
