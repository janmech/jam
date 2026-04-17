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
#include "c74_min.h"
#include "../jam.dmxusbpro.connector/jam.dmxusbpro.connector.hpp"
#include "../jam.helper/attribute_args_helper.hpp"


#define OBJECT_MESSAGE_PREFIX                "jam.dmxusbpro • "


using namespace c74::min;
namespace s_chrono = std::chrono;

class dmxusbpro : public object<dmxusbpro>
{
    
protected:
    
    bool _blackout            = false;
    std::atomic<bool> _io_threads_continue{false};
    std::atomic<bool> _close_device_requested = false;
    std::thread _receive_thread;
    std::thread _send_thread;
    std::mutex _open_device_lock;
    std::mutex _enqueue_msg_lock;
    std::mutex _messages_to_device_queue_lock;
    bool _accepting_messages_to_device = true;
    std::queue<std::vector<unsigned char> > _messages_to_device_queue;
    std::string _open_device_name = "";
    fifo<atoms> _to_max_queue { 1000 };
    unsigned char _dmx_universe[512];
    unsigned char _dmx_blackout[512];
    unsigned char _serial_in_buffer[SERIAL_IN_BUFF_SIZE];
    
    
    
    // for _receiveThreadTask()
    std::vector<unsigned char>          _device_response;
    bool                                _is_parsing_response      = false;
    int                                 _response_data_byte_count = 0;
    int                                 _respose_length           = 0;
    int                                 _response_index           = 0;
    s_chrono::steady_clock::time_point  _response_paring_start    = s_chrono::steady_clock::now();
    
    // for _sendThreadTask()
    std::condition_variable _messages_to_device_queue_cv;
    
    // for  _processDeviceResponds
    std::vector<unsigned char> _last_dmx_package;
    
    c74::max::t_object * _manager = nullptr;
    
    Connector * _deviceConnector = nullptr;
    
    Connector * _getConnector() {
        if(this->_deviceConnector == nullptr) {
            this->_deviceConnector = (Connector *)typedmess(this->_manager,symbol("get_connector"),0,0L);
        }
        return this->_deviceConnector;
    }
    
    void _enqueue_msg_to_max(const atoms &msg_to_max) {
        std::lock_guard<std::mutex> lock(_enqueue_msg_lock);
        this->_to_max_queue.try_enqueue(msg_to_max);
    }
    
    bool _dequeue_msg_to_max(atoms &msg_data) {
        std::lock_guard<std::mutex> lock(_enqueue_msg_lock);
        bool result = this->_to_max_queue.try_dequeue(msg_data);
        return result;
    }
    
    void _setOpenDeviceName(std::string device_name) {
        std::lock_guard<std::mutex> lock(_open_device_lock);
        this->_open_device_name = device_name;
    }
    
    std::string _getOpenDeviceName() {
        std::lock_guard<std::mutex> lock(_open_device_lock);
        return this->_open_device_name;
    }
    
    void _pushMessageToDevice(std::vector<unsigned char> message) {
        if(_accepting_messages_to_device) {
            {
            std::lock_guard<std::mutex> lock(_messages_to_device_queue_lock);
            this->_messages_to_device_queue.push(message);
            }
            _messages_to_device_queue_cv.notify_all();
        }
    }
    
    void _executeCloseDevice() {
        
        if(!this->initialized()) {
            return;
        }
        if(!this->_getConnector()->isConnected(this->_getOpenDeviceName())) {
            return;
        }
        // Stop accecpting new DMX messages
        _accepting_messages_to_device = false;
        if(!keepsending) {
            // clear queue
            {
                std::lock_guard<std::mutex> lock(_messages_to_device_queue_lock);
                std::queue<std::vector<unsigned char>> empty_queue;
                std::swap(_messages_to_device_queue, empty_queue);
            }
            _messages_to_device_queue_cv.notify_all();
            
            // Push the "Receive Mode" message, bypassing _pushMessageToDevice
            {
                std::lock_guard<std::mutex> lock(_messages_to_device_queue_lock);
                this->_messages_to_device_queue.push(std::vector<unsigned char> {
                    MSG_START_CONDITION,
                    MSG_LABEL_RECEIVE_DMX,
                    0x01, 0x00, 0x00,
                    MSG_END_CONDITION
                });
            }
            _messages_to_device_queue_cv.notify_all();
            
        }
        
        _close_device_requested = true;
        _messages_to_device_queue_cv.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if (_send_thread.joinable()) {
            _send_thread.join();
        }
           
        if (_receive_thread.joinable()) {
            _receive_thread.join();
        }
            
        
        if (this->_getConnector()->closeSerialPort(this->_getOpenDeviceName()) != 0) {
            cerr << "Error closing serial port." << endl;
        }
        this->_setOpenDeviceName("");
        
        
        atoms connection_state;
        
        connection_state.push_back(TO_OUTLET_2);
        connection_state.push_back(0);
        _enqueue_msg_to_max(connection_state);
        deliverer_to_max.delay(0);
        _close_device_requested = false;
        _accepting_messages_to_device = true;
    }
    
    void _sendThreadTask() {
        atoms to_max;
        if(!this->_getConnector()->isConnected(this->_getOpenDeviceName())) {
            return;
        }
            // Test if the device is still connected
        if (this->_getConnector()->connectionState(this->_getOpenDeviceName()) != Connector::ConnectionState::OK) {
            return;
        }
        
        std::vector<unsigned char> msg_bytes;
        
        
        {
            std::unique_lock<std::mutex> lock(_messages_to_device_queue_lock);
            
            _messages_to_device_queue_cv.wait(lock, [&]{
                return !_messages_to_device_queue.empty() || _close_device_requested;
                
            });
        
        }
        {
            std::lock_guard<std::mutex> lock(_messages_to_device_queue_lock);
            if (_close_device_requested && _messages_to_device_queue.empty()) {
                this->_io_threads_continue = false;
                return;
            }
            msg_bytes = std::move(_messages_to_device_queue.front());
            _messages_to_device_queue.pop();
        }
        _messages_to_device_queue_cv.notify_all();
        
        std::size_t success = write(
            this->_getConnector()->getFd(this->_getOpenDeviceName()),
            msg_bytes.data(),
            msg_bytes.size()
        );
        
        if(success < 0) {
            to_max.clear();
            to_max.push_back(TO_OUTLET_DUMPOUT);
            to_max.push_back("Error writing bytes");
            _enqueue_msg_to_max(to_max);
            deliverer_to_max.delay(0);
        }
    }
    
    void _receiveThreadTask() {
        
        if(this->_getConnector()->isConnected(this->_getOpenDeviceName())) {
                // Test if the device conntion is healthy
            int         connectionState = this->_getConnector()->connectionState(this->_getOpenDeviceName());
            
            if (connectionState != Connector::ConnectionState::OK) {
                switch (connectionState) {
                    case Connector::ConnectionState::MISSING:
                        cerr << "Device disconnected" << endl;
                        break;
                        
                    default:
                        cerr << "Device connection modified. Disconnecting" << endl;
                        break;
                }
                _is_parsing_response      = false;
                _response_data_byte_count = 0;
                _respose_length           = 0;
                _response_index           = 0;
                _io_threads_continue      = false;
                return;
            }
            
                // Getting response
            memset(_serial_in_buffer, 0, SERIAL_IN_BUFF_SIZE);
            
            std::size_t byte_count = read(this->_getConnector()->getFd(this->_getOpenDeviceName()), _serial_in_buffer, SERIAL_IN_BUFF_SIZE);
            
            if(_is_parsing_response) {
                    // Fallback: timeout if response isn't received completly within 200ms
                auto elapsed_parsing_time =
                s_chrono::duration_cast<s_chrono::milliseconds>(s_chrono::steady_clock::now() - _response_paring_start);
                
                if(elapsed_parsing_time.count() > RESPONSE_TIMEOUT) {
                    _is_parsing_response = false;
                    _response_index      = 0;
                    if(verbose) {
                        cwarn << "timeout receiving device response" << endl;
                    }
                }
            }
            
            if (byte_count > 0) {
                if(_serial_in_buffer[0] == MSG_START_CONDITION && !_is_parsing_response) {
                    _response_paring_start      = s_chrono::steady_clock::now();
                    _response_index             = 0;
                    _device_response.clear();
                    _is_parsing_response        = true;
                    _response_data_byte_count   = (int)(((std::uint16_t)(_serial_in_buffer[3]) << 8) | (std::uint16_t)_serial_in_buffer[2]);
                    _respose_length             = _response_data_byte_count + 5;
                }
                
                for (std::size_t i = 0; i < byte_count; i++) {
                    _device_response.push_back(_serial_in_buffer[i]);
                    _response_index++;
                }
                
                if(_response_index == _respose_length) {
                    atoms bytes_received;
                    
                    _is_parsing_response = false;
                    _response_index      = 0;
                    _processDeviceResponds(_device_response);
                }
            }
        }
    }
    
    void _processDeviceResponds(const std::vector<unsigned char> received_bytes) {
        atoms                             response_message;
        int                               breaktime_val;
        int                               mabtime_val;
        int                               refresh_val;
        char                              firmware_version[8];
        char                              serial_number_string[10];
        std::vector<unsigned char>        current_dmx_package;
        
        switch (received_bytes[1]) {
            case MSG_LABEL_GET_WIDGET_PARAMETRES:
                
                breaktime_val = (int)((float)received_bytes[6] * 10.67f);
                mabtime_val   = (int)((float)received_bytes[7] * 10.67f);
                refresh_val   = (int)received_bytes[8];
                
                snprintf(firmware_version, 8, "%d.%d", received_bytes[5], received_bytes[4] );
                response_message.push_back(TO_OUTLET_DUMPOUT);
                response_message.push_back("firmware");
                response_message.push_back(std::string(firmware_version));
                _enqueue_msg_to_max(response_message);
                deliverer_to_max.delay(0);
                
                response_message.clear();
                response_message.push_back(TO_OUTLET_DUMPOUT);
                response_message.push_back("breaktime");
                response_message.push_back(breaktime_val);
                _enqueue_msg_to_max(response_message);
                deliverer_to_max.delay(0);
                
                response_message.clear();
                response_message.push_back(TO_OUTLET_DUMPOUT);
                response_message.push_back("mabtime");
                response_message.push_back(mabtime_val);
                _enqueue_msg_to_max(response_message);
                deliverer_to_max.delay(0);
                
                response_message.clear();
                response_message.push_back(TO_OUTLET_DUMPOUT);
                response_message.push_back("refresh");
                response_message.push_back(refresh_val);
                _enqueue_msg_to_max(response_message);
                deliverer_to_max.delay(0);
                return;
                
            case MSG_LABEL_GET_WIDGET_SERIAL_NUMBER:
                snprintf(serial_number_string,10,"%02X%02X%02X%02X",
                         received_bytes[7], received_bytes[6],
                         received_bytes[5], received_bytes[4]
                         );
                response_message.push_back(TO_OUTLET_DUMPOUT);
                response_message.push_back("serialnumber");
                response_message.push_back(serial_number_string);
                _enqueue_msg_to_max(response_message);
                deliverer_to_max.delay(0);
                return;
                
            case MSG_LABEL_RECEIVED_DMX_PACKET:
                response_message.clear();
                current_dmx_package.clear();
                
                if(received_bytes[4] != 0) {
                    response_message.push_back(TO_MAX_CONSOLE_WARN);
                    response_message.push_back("corrupt DMX package received.");
                } else {
                    int         data_byte_count = (int)(((std::uint16_t)(received_bytes[3]) << 8) | (std::uint16_t)received_bytes[2]);
                    std::string out_format      = outformat.get();
                    std::string out_mode        = outmode.get();
                    
                    response_message.push_back(TO_OUTLET_1);
                    current_dmx_package.clear();
                    
                    std::size_t i = (out_format == "list") ? 2 : 1;
                    
                    for (; i < data_byte_count; i++) {
                        current_dmx_package.push_back(received_bytes[i + 4]);
                        
                        if(out_format == "list") {
                            response_message.push_back(i - 1);
                            response_message.push_back(received_bytes[i + 4]);
                        } else {
                            response_message.push_back(received_bytes[i + 4]);
                        }
                    }
                    
                    if(current_dmx_package != _last_dmx_package || out_mode == "always") {
                        _last_dmx_package = current_dmx_package;
                        _enqueue_msg_to_max(response_message);
                        deliverer_to_max.delay(0);
                    }
                }
                
                return;
                
            default:
                cerr << "error parsing device response." << endl;
                return;
        }
    }
    
    void _enqueMsgSendDmxPpacket(const unsigned char (&universe)[512]) {
        std::uint16_t              channel_count       = 512;
        std::uint16_t              data_byte_count     = channel_count + 1;
        unsigned char              data_byte_count_lsb = (unsigned char)(data_byte_count & 0x00FF);
        unsigned char              data_byte_count_msb = (unsigned char)((data_byte_count & 0xFF00) >> 8);
        
        {
            std::lock_guard<std::mutex> lock(_messages_to_device_queue_lock);
            if(_messages_to_device_queue.size() > 5) {
                if(verbose) {
                    cwarn << "reciving messages to fast: DMX packege dropped." << endl;
                }
                return;
            }
        }
        
        std::vector<unsigned char> msg_send_dmx {
            MSG_START_CONDITION,
            MSG_LABEL_SEND_DMX_PACKET,
            data_byte_count_lsb, data_byte_count_msb,
            0x00 // Start Code: USITT Default Null Start Code for Dimmers per DMX512 & DMX512/1990
        };
        
        for(std::uint16_t i = 0; i < channel_count; i++) {
            msg_send_dmx.push_back(universe[i]);
        }
        
        msg_send_dmx.push_back(MSG_END_CONDITION);
        
        this->_pushMessageToDevice(msg_send_dmx);
        
        
    }
    
public:
    
    dmxusbpro(const atoms& args = {}) {
        if(!dummy()) {
            memset(this->_dmx_universe, 0, 512);
            memset(this->_dmx_blackout, 0, 512);
            this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.dmxusbpro.manager"), 0, NULL);
            this->_deviceConnector = (Connector *)typedmess(this->_manager,symbol("get_connector"),0,0L);
        }
        
    }
    
    ~dmxusbpro() {
        if(!dummy()) {
            this->_executeCloseDevice();
        }
    }
    
    MIN_DESCRIPTION     { "Connect to the ENTTEC DMX USB Pro interface. Conrol DMX data with lists. <br/><i>The recommended firmware version is 1.44</i>" };
    
    MIN_TAGS            { "DMX control" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.dmxusbpro~, serial"};
    
    inlet<> input_1    { this, "(anything) Control Messages", "anything" };
    outlet<> output_1   { this, "(list) DMX Output <startcode> <channel> <value>", "list" };
    outlet<> output_2   { this, "(int) State of Connection", "int" };
    outlet<> output_3   { this, "(anything) Connect to umenu" };
    outlet<> output_dumpout   { this, "dumpout"};
    
    timer<> deliverer_to_max {
        this, MIN_FUNCTION {
            atoms queue_data;
            
            while (_dequeue_msg_to_max(queue_data)) {
                int   destination = queue_data[0];
                atoms message;
                
                for(std::size_t i = 1; i < queue_data.size(); i++) {
                    message.push_back(queue_data[i]);
                }
                
                switch (destination) {
                    case TO_OUTLET_1:
                        output_1.send(message);
                        break;
                        
                    case TO_OUTLET_2:
                        output_2.send(message);
                        break;
                        
                    case TO_OUTLET_3:
                        output_3.send(message);
                        break;
                        
                    case TO_OUTLET_DUMPOUT:
                        output_dumpout.send(message);
                        break;
                        
                    case TO_MAX_CONSOLE:
                        
                        for(std::size_t i = 0; i < message.size(); i++) {
                            cout << message[i] << endl;
                        }
                        
                        break;
                        
                    case TO_MAX_CONSOLE_WARN:
                        
                        for(std::size_t i = 0; i < message.size(); i++) {
                            cout << message[i] << endl;
                        }
                        
                        break;
                }
            }
            return {};
        }
    };
    
    attribute<bool> verbose {
        this,
        "verbose",
        false,
        title { "Verbose" },
        description { "If set to 0 (default), only serial devices following the ENTTEC USB DMX Pro naming convention will be enabled in a umenu connected to the third outlet. <br /> If set to 1 all serial devices will be enabled and more information about the coinnection state will be printed to the Max console." },
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
            return cleaned_args;
        }},
    };
    
    attribute<bool> keepsending {
        this, "keepsending", true,
        title { "Keep sending" },
        description { "If set to 0, the device will stop sending DMX data when the connection is closed. If set to 1 (default) the device will continue to send the last received DMX data after the connection has been closed." },
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, true);
            return cleaned_args;
        }},
    };

    attribute<int> baudrate {
        this, "baudrate", 56700,
        title {"Device Baud Rate"},
        description{"Set the baud rate for communicating with the interface. Default: 56700"},
        range {9600, 256000},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<int>(args, &cleaned_args, 1, 56700);
                int value = (int)cleaned_args[0];
                value = std::clamp(value, 9600, 256000);
                cleaned_args[0] = value;
                if(this->initialized()) {
                    if(this->_getConnector()->isConnected(this->_getOpenDeviceName())) {
                        cerr << "baudrate has changed. closing the connection." << endl;
                        this->_executeCloseDevice();
                    }
                }
                return cleaned_args;
            }
        }
    };
    
    attribute<symbol> outformat {
        this, "outformat", "list",
        title { "DMX data output format" },
        description { "DMX data format sent to first outlet. If set to 'list' (default) a list of pairs of <i>DMX Channel</i> and <i>DMX Value</i> will be send out. If set to 'raw' outputs the raw bytes of received DMX data:<br />First byte: Status code. Following Bytes: DMX values.<br />" },
        range {"raw", "list"},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<std::string>(args, &cleaned_args, 1, "list");
                std::string value = cleaned_args[0];
                if(value != "list" && value != "raw") {
                    cleaned_args[0] = "list";
                }
                return cleaned_args;
            }
        }
    };
    
    attribute<symbol> outmode {
        this, "outmode", "onchange",
        title { "DMX data output mode" },
        description { "If set to 'onchange' (default) first outlet will only send out DMX data if values have changed. <bvr />If set to 'always' every received DMX package will we sent out the first outlet." },
        range {"onchange", "always"},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<std::string>(args, &cleaned_args, 1, "onchange");
                std::string value = cleaned_args[0];
                if(value != "onchange" && value != "always") {
                    cleaned_args[0] = "onchange";
                }
                return cleaned_args;
            }
        }
        
    };
    
    message<threadsafe::yes> receive {
        this, "receive",
        "Set device to receive DMX messages.<br /><b>Note</b>: Sending a list of DMX values or sending the message deviceserial will set the device into send mode again.",
        MIN_FUNCTION {
            
            if(!this->_getConnector()->isConnected(this->_getOpenDeviceName())) {
                if(verbose) {
                    cerr << "Can't set receive mode, not connected." << endl;
                }
                
                return {};
            }
            this->_pushMessageToDevice(std::vector<unsigned char> {
                MSG_START_CONDITION,
                MSG_LABEL_RECEIVE_DMX,
                0x01, 0x00, 0x00,
                MSG_END_CONDITION
            });
            
            return {};
        }
    };
    
    message<threadsafe::yes> open {
        this, "open", "Open serial connection to a device.",
        MIN_FUNCTION {
            if (args.size() > 1) {
                cwarn << "extra argument for message 'open'" << endl;
            }
            
            if (args.size() < 1) {
                cwarn << "missing argument for message 'open'" << endl;
                return {};
            }
            
            std::string device_name = args[0];
            
            if(this->_open_device_name != device_name) {
                if(this->_getConnector()->isConnected(device_name)) {
                    cerr << "'" << device_name << "' already opened by another instance." << endl;
                    return {};
                }
            }
            
            if(verbose) {
                cout << "opening " + device_name << endl;
            }
            
            if(!this->_getConnector()->deviceExists(device_name)) {
                cerr << "specified port not available" << endl;
                return {};
            }
            this->_executeCloseDevice();
            
            int         set_baudrate = baudrate;
            int         open_success = this->_getConnector()->openSerialPort(device_name, set_baudrate);
            
            if(open_success == -1) {
                cerr << "Error opening device" << endl;
                return {};
            }
            
            if(open_success == -2) {
                cerr << "'" << device_name << "' already opened by another instance." << endl;
                return {};
            }
            
            this->_setOpenDeviceName(device_name);
            atoms       connection_state;
            connection_state.push_back(TO_OUTLET_2);
            connection_state.push_back(1);
            _enqueue_msg_to_max(connection_state);
            deliverer_to_max.delay(0);
            
            this->_io_threads_continue = true;
            // std::this_thread::sleep_for(s_chrono::milliseconds(10));
            
            this->_receive_thread = std::thread([this]()
                                                {
            atoms msg_to_console;
            
            if (verbose) {
                msg_to_console.push_back(TO_MAX_CONSOLE);
                msg_to_console.push_back("starting receive thread");
                _enqueue_msg_to_max(msg_to_console);
                deliverer_to_max.delay(0);
            }
            
            while(_io_threads_continue) {
                _receiveThreadTask();
            }
            
            if (verbose) {
                msg_to_console.clear();
                msg_to_console.push_back(TO_MAX_CONSOLE);
                msg_to_console.push_back("stopping receive thread");
                _enqueue_msg_to_max(msg_to_console);
                deliverer_to_max.delay(0);
            }
            });
            
            this->_send_thread = std::thread([this]()
                                             {
            atoms msg_to_console;
            
            if (verbose) {
                msg_to_console.push_back(TO_MAX_CONSOLE);
                msg_to_console.push_back("starting send thread");
                _enqueue_msg_to_max(msg_to_console);
                deliverer_to_max.delay(0);
            }
            
            while(_io_threads_continue) {
                _sendThreadTask();
            }
            
            if (verbose) {
                msg_to_console.clear();
                msg_to_console.push_back(TO_MAX_CONSOLE);
                msg_to_console.push_back("stopping send thread");
                _enqueue_msg_to_max(msg_to_console);
                deliverer_to_max.delay(0);
            }
            });
                //this->_send_thread.detach();
            return {};
        }
    };
    
    message<threadsafe::yes> menu {
        this, "menu", "Get list of connected devices and build menu from it.",
        MIN_FUNCTION {
            if (args.size() > 1) {
                cwarn << "extra argument for message 'menu'" << endl;
            }
            
            std::vector<std::string> serial_devices;
            atoms                    out_atoms;
            
            output_3.send("clear");
            out_atoms.push_back("append");
            out_atoms.push_back("(Select Interface)");
            output_3.send(out_atoms);
            
            std::vector<std::string> device_names = this->_getConnector()->getDeviceNames(verbose, true);
            
            for (auto& device_name : device_names) {
                atoms device_list;
                device_list.push_back("append");
                device_list.push_back(device_name);
                output_3.send(device_list);
            }
            
            return {};
        }
    };
    
    message<threadsafe::yes> list {
        this, "list", "An even number of integers, indicating pairs of <i>DMX Channel</i> and <i>DMX Value</i>.<br/>Sets the specified channels to the specified values.",
        MIN_FUNCTION {
            if(!this->_getConnector()->isConnected(this->_getOpenDeviceName())) {
                return{};
            }
            
                //Are all arguments integers?
            for(std::size_t i = 0; i < args.size(); i++) {
                if(args[i].type() != message_type::int_argument) {
                    cerr << "Invalid arument type. Expeting list on integers." << endl;
                    return {};
                }
            }
            
                // Checking argument count
            if(args.size() % 2 != 0) {
                cerr << "Invalid argument count for message list. Expecting pairs of integers (0-255): DMX Channel | DMX Value." << endl;
                return {};
            }
            
            if(args[1].type() != message_type::int_argument) {
                cerr << "invalid argument type for second list element. expecting int." << endl;
                return {};
            }
            
            for(std::size_t i = 0; i < args.size(); i = i + 2) {
                int dmx_channel = args[i];
                int dmx_val     = args[i + 1];
                dmx_channel = std::min(512, std::max(1, dmx_channel));
                dmx_val     = std::min(255, std::max(0, dmx_val));
                
                    // setting the channel in the universe
                this->_dmx_universe[dmx_channel - 1] = (unsigned char)dmx_val;
            }
            
            if(!this->_blackout) {
                this->_enqueMsgSendDmxPpacket(this->_dmx_universe);
            }
            
            return {};
        }
    };
    
    message<threadsafe::yes> devicesettings {
        this, "devicesettings",
        "Read firmware version, breaketime, MAB time and refresh rate from the device and send it out the rightmost outlet.",
        MIN_FUNCTION {
            if(args.size() > 0) {
                cwarn << "extra argument for message 'getparams'" << endl;
            }
            
            if(!this->_getConnector()->isConnected(this->_getOpenDeviceName())) {
                if(verbose) {
                    cerr << "Can't get DMX parameters, not connected." << endl;
                }
                
                return {};
            }
            this->_pushMessageToDevice(std::vector<unsigned char> {
                MSG_START_CONDITION,
                MSG_LABEL_GET_WIDGET_PARAMETRES,
                0x00, 0x00,
                MSG_END_CONDITION
            });
            return {};
        }
    };
    
    message<threadsafe::yes> deviceserial {
        this, "deviceserial", "Get the devices serial number.<br/><b>NOTE:</b> If the device is currently outputting DMX data, sending this message will set it back to send mode",
        MIN_FUNCTION {
            if(args.size() > 0) {
                cwarn << "extra argument for message 'getserial'" << endl;
            }
            
            if(!this->_getConnector()->isConnected(this->_getOpenDeviceName())) {
                if(verbose) {
                    cerr << "Can't get serial number, not connected." << endl;
                }
                
                return {};
            }
            this->_pushMessageToDevice(std::vector<unsigned char> {
                MSG_START_CONDITION,
                MSG_LABEL_GET_WIDGET_SERIAL_NUMBER,
                0x00, 0x00,
                MSG_END_CONDITION
            });
            return {};
            
        }
        
    };
    
    message<threadsafe::yes> close {
        this, "close", "Close the device connection. If <i>keepsending</i> is 0: Stop sending DMX data.",
        MIN_FUNCTION {
            if (args.size() > 1) {
                cwarn << "extra argument for message 'close'" << endl;
                return {};
            }
            
            this->_executeCloseDevice();
            return{};
        }
    };
    
    message<threadsafe::yes> blackout {
        this, "blackout", "Set all DMX channels temporarily to 0.",
        MIN_FUNCTION {
            if(!this->_getConnector()->isConnected(this->_getOpenDeviceName())) {
                if(verbose) {
                    cerr << "Cannot set blackout, not connected" << endl;
                }
                
                return {};
            }
            
            if (args.size() > 1) {
                cwarn << "extra argument for message 'blackout'" << endl;
                return {};
            }
            
            if(args[0].type() != message_type::int_argument) {
                cerr << "Invalid argument type for message 'blackout. Expected a boolean" << endl;
                return {};
            }
            
            this->_blackout = !(((int)args[0]) == 0);
            
            if(this->_blackout) {
                this->_enqueMsgSendDmxPpacket(this->_dmx_blackout);
            } else {
                this->_enqueMsgSendDmxPpacket(this->_dmx_universe);
            }
            
            return{};
        }
    };
};


MIN_EXTERNAL(dmxusbpro);
