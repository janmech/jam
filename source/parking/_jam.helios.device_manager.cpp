    //
    //  jam.helios.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 12/3/25.
    //

#include "./jam.helios.device_manager.hpp"

namespace jam::helios {
    HeliosDac * DeviceManager::getDac() {
        return &this->_helios_dac;
    };
    int DeviceManager::deviceScan() {
        this->_is_scanning = true;
        this->_helios_dac.CloseDevices();
        
        this->_open_dev_lock.lock();
        this->_open_devices->clear();
        this->_open_dev_lock.unlock();
        
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
                
                this->_open_dev_lock.lock();
                this->_open_devices->push_back(dac);
                this->_open_dev_lock.unlock();
            }
        }
        this->_is_scanning = false;
        return numDevs;
    }
    
    std::vector<device_info_t> *DeviceManager::getOpenDevices() {
        
        this->_open_dev_lock.lock();
        std::vector<device_info_t> *open_devs = this->_open_devices;
        this->_open_dev_lock.unlock();
        return open_devs;
    };
    
    bool DeviceManager::isScanning() {
        return this->_is_scanning;
    }
    
    void DeviceManager::addObjInstance(c74::max::t_object *intstance) {
        if (std::find(this->_object_instances.begin(), this->_object_instances.end(), intstance) == this->_object_instances.end()) {
            
                this->_open_dev_lock.lock();
                this->_object_instances.push_back(intstance);
                this->_open_dev_lock.unlock();
        }
    };
    
    void DeviceManager::removeObjInstance(c74::max::t_object *intstance) {
        
        this->_object_instances.erase(
                                      std::remove(this->_object_instances.begin(),this->_object_instances.end(), intstance),
                                      this->_object_instances.end()
                                      );
    };
    
    void DeviceManager::notifyInstances() {
        for(size_t i = 0; i < this->_object_instances.size(); i++) {
            c74::max::typedmess(this->_object_instances[i], c74::max::gensym("menu"), 0, 0L);
        }
    };
    
    std::string DeviceManager::getTypeName(DacType type) {
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
    
    DeviceState DeviceManager::attachDeviceToInstance(int device_index, c74::max::t_object *intstance) {
        try {
            device_info_t* device = this->_findDevice(device_index);
            return this->_attachDevice(device, intstance);
        } catch (DeviceState e) {
            return e;
        }
    };
    
    DeviceState DeviceManager::attachDeviceToInstance(std::string device_name, c74::max::t_object *intstance) {
        try {
            device_info_t* device = this->_findDevice(device_name);
            return this->_attachDevice(device, intstance);
        } catch (DeviceState e) {
            return e;
        }
        
    };
    
    bool DeviceManager::detachDeviceFromInstance(c74::max::t_object *intstance) {
        bool was_attached = false;
        std::vector<device_info_t> * devs = this->getOpenDevices();
        for(size_t i = 0; i < (*devs).size(); i++) {
            if((*devs)[i].owner == intstance) {
                (*devs)[i].owner = NULL;
                was_attached = true;
            }
        }
        return was_attached;
    };
    
    void DeviceManager::setShutter( c74::max::t_object *intstance, bool state) {
        std::vector<device_info_t> * devs = this->getOpenDevices();
        this->_open_dev_lock.lock();
        for(size_t i = 0; i < (*devs).size(); i++) {
            if((*devs)[i].owner == intstance) {
                this->_helios_dac.SetShutter((unsigned int)(*devs)[i].index, state);
            }
        }
        this->_open_dev_lock.unlock();
    };


    /* Protected methods*/
    
    DeviceState DeviceManager::_attachDevice(device_info_t *device, c74::max::t_object *intstance) {
        try {
            if(device->owner != NULL && device->owner != intstance) {
                throw DeviceState::ATTACHED_ERROR_ALREADY_ATTACHED;
            }
            device->owner = intstance;
        } catch(DeviceState e)  {
            return  e;
        }
        return DeviceState::ATTACHED_SUCCESS;
    };
    
    device_info_t* DeviceManager::_findDevice(std::string name) {
        std::vector<device_info_t> * devs = this->getOpenDevices();
        for(size_t i = 0; i < (*devs).size(); i++) {
            if(((*devs))[i].name == name) {
                return &(*devs)[i];
            }
        }
        throw DeviceState::NOTFOUND;
        
    };
    
    device_info_t*  DeviceManager::_findDevice(int index) {
        std::vector<device_info_t> * devs = this->getOpenDevices();
        for(size_t i = 0; i < (*devs).size(); i++) {
            if(((*devs))[i].index == index) {
                return &(*devs)[i];
            }
        }
        throw DeviceState::NOTFOUND;
    };
    
}
