    /// @file
    ///	@ingroup    jam
    ///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
    ///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include "c74_min.h"
#include "helios-sdk/cpp/HeliosDac.h"
#include "helios-sdk/cpp/libusb.h"
#include "jam.helios.device_manager.hpp"

#define OBJECT_VERSION "jam.helios v.0.0.0"
#define HELIOS_FILE_CHUNK 1024

using namespace c74::min;

class helios : public object<helios>
{
    
protected:
    typedef struct QuededMessage {
        outlet<>* out;
        atoms msg_atoms;
        void set(outlet<>* o, atoms ma) {
            this->out = o;
            this->setAtoms(ma);
        }
        
        void setAtoms(atoms ma) {
            this->msg_atoms.clear();
            this->msg_atoms = ma;
        }
        
        void send(helios* me) {
            me->_enqueue_msg_to_max(*this);
            me->deliverer_to_max.delay(0);
        }
    } queued_message_t;
    
    std::thread _device_scan_thread;
    fifo<queued_message_t> _to_max_queue_2{ 1000 };
    std::mutex _enqueue_msg_lock;
    jam::helios::DeviceManager& _deviceManager = jam::helios::DeviceManager::get();
    
    void _enqueue_msg_to_max(const queued_message_t& msg_to_max) {
        _enqueue_msg_lock.lock();
        this->_to_max_queue_2.try_enqueue(msg_to_max);
        _enqueue_msg_lock.unlock();
    }
    
    bool _dequeue_msg_to_max(queued_message_t& msg_data) {
        _enqueue_msg_lock.lock();
        bool result = this->_to_max_queue_2.try_dequeue(msg_data);
        _enqueue_msg_lock.unlock();
        return result;
    }
    
public:
    helios(const atoms& args = {}) {
        if (!dummy()) {
            this->_deviceManager.addObjInstance(this->maxobj());
        }
    }
    
    ~helios() {
        if (!dummy()) {
            this->_deviceManager.removeObjInstance(this->maxobj());
        }
    }
    
    MIN_DESCRIPTION { "Connect to a Helios ILDA DAC" };
    MIN_TAGS { "laser control" };
    MIN_AUTHOR{ "Jan Mech" };
    MIN_RELATED{ "jam.dmxusbpro~, jam.dmxusbpro" };
    
    inlet<> input_1{ this, "(anything) Control Messages", "anything" };
    inlet<> input_2{ this, "(dictionary) ilda file dictionary", "dictionary" };
    outlet<> outlet_menu{ this, "(anything) Connect to umenu", "message" };
    outlet<> outlet_connected{ this, "(int) State of Connection", "int" };
    outlet<> outlet_dumpout{ this, "dumpout" };
    
    attribute<bool> notifyothers{
        this, "notifyothers", false,
        title{ "Notify others" },
        description{ "If set to 1 other jam.helios object will be notified if an  device scan has been exectued. The new result will update all umenus connected to the leftmost outlet. Default: 0" }
    };
    
    attribute<int, threadsafe::no, limit::clamp> samplerate {
        this,
        "samplerate",
        30000,
        title{ "Samplerate" },
        description{ "Points per second send to the laser projector.<br /><b>Note</b>:It is recommended to keep sampling rate at 30000 or below, as higher values can cause problems in certain devices like LaserCube Wifi." },
        range{ 1000, 100000 },
    };
    
    attribute<bool> invert_x {
        this, "invert_x", false,
        title{ "Invert X" },
        description{ "Invert the output of the X-axis (horizontally)" }
    };
    
    attribute<bool> invert_y {
        this, "invert_y", false,
        title{ "Invert Y" },
        description{ "Invert the output of the Y-axis (vertically)" }
    };
    
    message<> menu {
        
        this, "menu", "Get list of connected devices and build menu from it.",
        MIN_FUNCTION{
            this->close();
            if (this->_deviceManager.getOpenDevices()->size() == 0) {
                cwarn << "No devices registered. Try re-scanning." << endl;
            }
            if (args.size() > 1) {
                cwarn << "extra argument for message 'menu'" << endl;
            }
            std::vector<jam::helios::device_info_t>* open_devices = this->_deviceManager.getOpenDevices();
            atoms msg_atoms;
            msg_atoms.push_back("clear");
            queued_message_t msg;
            msg.set(&outlet_menu, msg_atoms);
            msg.send(this);
            
            msg_atoms.clear();
            msg_atoms.push_back("append");
            msg_atoms.push_back("(Select Interface)");
            msg.setAtoms(msg_atoms);
            msg.send(this);
            
            msg_atoms.clear();
            for (size_t i = 0; i < open_devices->size(); i++) {
                msg_atoms.push_back("append");
                msg_atoms.push_back((*open_devices)[i].name);
                msg.setAtoms(msg_atoms);
                msg.send(this);
            }
            return {};
        }
    };
    
    message<> devicescan {
        this, "devicescan", "Scan for connected Helios DACs.",
        MIN_FUNCTION{
            this->close();
            if (args.size() > 1) {
                cwarn << "extra argument for message 'menu'" << endl;
            }
            if (this->_deviceManager.isScanning()) {
                cwarn << "scan already in progress" << endl;
                return {};
            }
            
            this->_device_scan_thread = std::thread([this]() {
                auto b = this->box();
                number current_progress{ -1. };
                b("startprogress", &current_progress);
                int numDevs = this->_deviceManager.deviceScan();
                atoms scan_result;
                scan_result.clear();
                scan_result.push_back(atom("devicescan"));
                scan_result.push_back(atom(numDevs));
                queued_message_t msg;
                msg.set(&outlet_dumpout, scan_result);
                msg.send(this);
                b("stopprogress");
                if (notifyothers) {
                    this->_deviceManager.notifyInstances();
                }
                else {
                    menu();
                }
            });
            this->_device_scan_thread.detach();
            
            return {};
        }
    };
    
    message<> deviceinfo {
        this, "deviceinfo", "Print infomation about Helios DAC devices to the Max console.",
        MIN_FUNCTION{
            std::vector<jam::helios::device_info_t>* devs = this->_deviceManager.getOpenDevices();
            if (devs->size() == 0) {
                cwarn << "No devices registered. Try re-scanning." << endl;
            }
            for (jam::helios::device_info_t info : *devs) {
                cout << "Device " << info.index + 1 << endl;
                cout << "    Name: " << info.name << endl;
                ;
                cout << "    Type: " << this->_deviceManager.getTypeName(info.type) << endl;
                ;
                cout << "    Firmware: " << info.firmware << endl;
            }
            return {};
        }
    };
    
    message<> open {
        this, "open", "Open connetion to a Helios DAC",
        MIN_FUNCTION{
            if (args.size() == 0){
                cwarn << "missing argument for message open" << endl;
                return {};
            }
            if (args.size() > 1) {
                cwarn << "extra argument for message open" << endl;
            }
            
            atom device_id = args[0];
            std::string dev_name = "";
            int dev_index = 0;
            bool id_is_name = false;
            if (device_id.a_type == c74::max::A_SYM) {
                dev_name = (std::string)device_id;
                id_is_name = true;
            }
            else {
                dev_index = (int)device_id;
                    // Publicly displayed device indices start with 1, internal inidices with 0. We need to take that into account.
                if (dev_index < 1) {
                    cwarn << "device not found" << endl;
                    return {};
                }
                dev_index--;
            }
            
            queued_message_t msg;
            atoms msg_atoms;
            auto result = jam::helios::DeviceState::NOTFOUND;
            
            if (id_is_name) {
                result = this->_deviceManager.attachDeviceToInstance(dev_name, this->maxobj());
            }
            else {
                result = this->_deviceManager.attachDeviceToInstance(dev_index, this->maxobj());
            }
            switch (result) {
                case jam::helios::DeviceState::ATTACHED_ERROR_ALREADY_ATTACHED:
                    cwarn << "device already opened by other instance" << endl;
                    break;
                case jam::helios::DeviceState::NOTFOUND:
                    cwarn << "device not found" << endl;
                    break;
                case jam::helios::DeviceState::ATTACHED_SUCCESS:
                    break;
                default:
                    cwarn << "error not opening device" << endl;
            }
            if (result != jam::helios::DeviceState::ATTACHED_SUCCESS) {
                msg_atoms.push_back(0);
            }
            else {
                msg_atoms.push_back(1);
            }
            msg.set(&outlet_connected, msg_atoms);
            msg.send(this);
            return {};
        }
    };
    
    message<> close {
        this, "close", "Close connetion to Helios DAC",
        MIN_FUNCTION{
            this->_deviceManager.detachDeviceFromInstance(this->maxobj());
            queued_message_t msg;
            atoms msg_atoms;
            msg_atoms.push_back(0);
            msg.set(&outlet_connected, msg_atoms);
            msg.send(this);
            return {};
        }
    };
    
    message<> shutter {
        this, "shutter", "Open/Close the shutter. <p><b>Argument:</b><br /> shutter_state [int]</p>",
        MIN_FUNCTION{
            if (args.size() < 1){
                cwarn << "missing argument for message shutter" << endl;
                return {};
            }
            if (args.size() > 1) {
                cwarn << "extra argument for message shutter" << endl;
            }
            int shutter_state_int = (int)args[0];
            bool shutter_state = shutter_state_int = !0;
            this->_deviceManager.setShutter(this->maxobj(), shutter_state);
            return {};
        }
    };
    
    timer<> deliverer_to_max {
        this, MIN_FUNCTION{
            queued_message_t queue_msg;
            
            while (_dequeue_msg_to_max(queue_msg)) {
                queue_msg.out->send(queue_msg.msg_atoms);
            }
            return {};
        }
    };
};

MIN_EXTERNAL(helios);
