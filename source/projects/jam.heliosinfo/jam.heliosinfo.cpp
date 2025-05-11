    /// @file
    ///	@ingroup    jam
    ///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
    ///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#include <cstddef>
#include <queue>
#include <vector>
#include <string>
#include "c74_min.h"
#include "../jam.helios.connector/jam.helios.connector.hpp"

using namespace c74::min;
using namespace jam::helios;

class heliosinfo : public object<heliosinfo>
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
        
        void send(heliosinfo* me) {
            me->_enqueue_msg_to_max(*this);
            me->deliverer_to_max.delay(0);
        }
    } queued_message_t;
    
    fifo<queued_message_t> _to_max_queue_2{ 1000 };
    std::mutex _enqueue_msg_lock;
    
    
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
    
    std::thread _device_scan_thread;
    
    c74::max::t_object * _manager = nullptr;
    
    Connector * _connector = nullptr;
    
    Connector * _getConnector() {
        if(this->_connector == nullptr) {
            this->_connector = (Connector *)typedmess(this->_manager,symbol("get_connector"),0,0L);
        }
        return this->_connector;
    }
    
    
    
public:
    heliosinfo(const atoms& args = {}) {
        if (!dummy()) {
            this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.helios.manager"), 0, NULL);
            this->_connector = (Connector *)typedmess(this->_manager,symbol("get_connector"),0,0L);
        }
    }
    
    ~heliosinfo() {
        if (!dummy()) {
            
        }
    }
    
    MIN_DESCRIPTION { "Connect to a Helios ILDA DAC" };
    MIN_TAGS { "laser control" };
    MIN_AUTHOR{ "Jan Mech" };
    MIN_RELATED{ "jam.dmxusbpro~, jam.dmxusbpro" };
    
    inlet<> input_1{ this, "(anything) Control Messages", "anything" };
    outlet<> outlet_menu { this, "(anything) Connect to umenu", "message" };
    outlet<> outlet_dumpout{ this, "dumpout" };
    
    message<> devicescan {
        this, "devicescan", "Scan for connected Helios DACs.",
        MIN_FUNCTION {
            if (args.size() > 1) {
                cwarn << "extra argument for message 'menu'" << endl;
            }
            if (this->_getConnector()->isScanning()) {
                cwarn << "scan already in progress" << endl;
                return {};
            }
            
            this->_device_scan_thread = std::thread([this]() {
                auto b = this->box();
                number current_progress{ -1. };
                b("startprogress", &current_progress);
                int numDevs = this->_connector->deviceScan();
                atoms scan_result;
                scan_result.clear();
                scan_result.push_back(atom("devicescan"));
                scan_result.push_back(atom(numDevs));
                queued_message_t msg;
                msg.set(&outlet_dumpout, scan_result);
                msg.send(this);
                b("stopprogress");
                menu();
            });
            this->_device_scan_thread.detach();
            
            return {};
        }
    };
    
    message<> menu {
        
        this, "menu", "Get list of connected devices and build menu from it.",
        MIN_FUNCTION{
            if (this->_getConnector()->getOpenDevices()->size() == 0) {
                cwarn << "No devices registered. Try re-scanning." << endl;
            }
            if (args.size() > 1) {
                cwarn << "extra argument for message 'menu'" << endl;
            }
            std::vector<jam::helios::device_info_t>* open_devices = this->_getConnector()->getOpenDevices();
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
    
    message<> info {
        this, "info", "Print infomation about Helios DAC devices to the Max console.",
        MIN_FUNCTION{
            if(args.size() > 1) {
                return {};
            }
            int device_index = args[0];
            if(device_index < 1) {
                return {};
            }
            
            device_index--;
            
            std::vector<device_info_t> * devices = this->_getConnector()->getOpenDevices();
            atoms msg_atoms;
            queued_message_t msg;
            
            try {
                device_info_t info = devices->at(device_index);
                cout << "device info:" << endl;
                cout << "    index: " << info.index + 1 << endl;
                cout << "    name: " << info.name << endl;
                cout << "    type: " << this->_getConnector()->getTypeName(info.type) << endl;
                cout << "    firmware: " << info.firmware << endl;
                
            } catch (const std::out_of_range& e) {
                cout << "device index out of range"<< endl;
                return {};
            }
            
            return {};
        }
    };
    
    
    timer<> deliverer_to_max {
        this, MIN_FUNCTION {
            queued_message_t queue_msg;
            
            while (_dequeue_msg_to_max(queue_msg)) {
                queue_msg.out->send(queue_msg.msg_atoms);
            }
            return {};
        }
    };
};

MIN_EXTERNAL(heliosinfo);
