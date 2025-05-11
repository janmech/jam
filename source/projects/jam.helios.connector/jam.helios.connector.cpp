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
};
