/// @file
///	@ingroup    jam
///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include "../jam.device_manager/jam.dmxusbpro.dmx_device.hpp"
#include "c74_min.h"

#define OBJECT_MESSAGE_PREFIX              "jam.dmxusbpro~ • "


using namespace c74::min;
namespace s_chrono = std::chrono;

class dmxusbpro_tilde : public object<dmxusbpro_tilde>, public vector_operator<>
{
    private:

        std::vector< std::unique_ptr<inlet<> > > _inlets;
        std::vector<int> _inlet_dmx_channel;

    protected:

        bool _io_threads_continue = false;
        std::thread _receive_thread;
        std::thread _send_thread;
        std::mutex _open_device_lock;
        std::mutex _enque_msg_lock;
        std::queue<std::vector<unsigned char> > _messages_to_device_queue;
        std::string _open_device_name = "";
        fifo<atoms> _to_max_queue { 1000 };
        unsigned char _dmx_universe[512];
        unsigned char _serial_in_buffer[SERIAL_IN_BUFF_SIZE];
        dict _connections { symbol("__jamproconnections__") }; // Workaround until I find a way to make the device manager global

        void _enque_msg_to_max(const atoms &msg_to_max) {
            _enque_msg_lock.lock();
            this->_to_max_queue.try_enqueue(msg_to_max);
            _enque_msg_lock.unlock();
        }

        bool _dequeue_msg_to_max(atoms &msg_data) {
            _enque_msg_lock.lock();

            bool result = this->_to_max_queue.try_dequeue(msg_data);

            _enque_msg_lock.unlock();
            return result;
        }

        void _setOpenDeviceName(std::string device_name) {
            this->_open_device_lock.lock();
            this->_open_device_name = device_name;
            this->_open_device_lock.unlock();
        }

        std::string _getOpenDeviceName() {
            return this->_open_device_name;
        }

        void _closeDevice() {
            if(Connector::get().isConnected(this->_getOpenDeviceName())) {

                if(!keepsending) {
                    this->_messages_to_device_queue.push(std::vector<unsigned char> {
                    MSG_START_CONDITION,
                    MSG_LABEL_RECEIVE_DMX,
                    0x01, 0x00, 0x00,
                    MSG_END_CONDITION
                });
                }

                // wait for message to be sent
                std::this_thread::sleep_for(s_chrono::milliseconds(50));

                this->_io_threads_continue = false;

                if (Connector::get().closeSerialPort(this->_getOpenDeviceName()) != 0) {
                    cerr << "Error closing serial port." << endl;
                }

                this->_connections[this->_open_device_name] = 0;
                this->_open_device_name                     = "";

                static_cast<void>(this->_messages_to_device_queue.empty());
            }


            atoms connection_state;

            connection_state.push_back(TO_OUTLET_2);
            connection_state.push_back(0);
            _enque_msg_to_max(connection_state);
            deliverer_to_max.delay(0);
        }

        void _sendThreadTask() {
            atoms to_max;

            if(Connector::get().isConnected(this->_getOpenDeviceName())) {
                // Test if the device is still connected
                if (Connector::get().connectionState(this->_getOpenDeviceName()) != Connector::ConnectionState::OK) {
                    return;
                }

                if(_messages_to_device_queue.size() > 0 ) {
                    std::vector<unsigned char> msg_bytes = _messages_to_device_queue.front();

                    _messages_to_device_queue.pop();

                    std::size_t                msg_size = msg_bytes.size();
                    char                       msg_buffer[msg_bytes.size()];

                    for(std::size_t i = 0; i < msg_bytes.size(); i++) {
                        msg_buffer[i] = msg_bytes[i];
                    }

                    std::size_t                success = write(Connector::get().getFd(this->_getOpenDeviceName()), msg_buffer, msg_size);

                    if(success < 0) {
                        to_max.clear();
                        to_max.push_back(TO_OUTLET_DUMPOUT);
                        to_max.push_back("Error writing bytes");
                        _enque_msg_to_max(to_max);
                        deliverer_to_max.delay(0);
                    }
                } else {
                    std::this_thread::sleep_for(s_chrono::milliseconds(5));
                }
            }
        }

        void _receiveThreadTask() {
            static std::vector<unsigned char> device_response;
            static bool                       is_parsing_response      = false;
            static int                        response_data_byte_count = 0;
            static int                        respose_length           = 0;
            static int                        response_index           = 0;
            static s_chrono::time_point       response_paring_start    = s_chrono::steady_clock::now();


            if(Connector::get().isConnected(this->_getOpenDeviceName())) {
                // Test if the device conntion is healthy
                int         connectionState = Connector::get().connectionState(this->_getOpenDeviceName());

                if ( connectionState != Connector::ConnectionState::OK) {
                    switch (connectionState) {
                        case Connector::ConnectionState::MISSING:
                            cerr << "Device disconnected" << endl;
                            break;

                        default:
                            cerr << "Device connection modified. Disconnecting" << endl;
                            break;
                    }
                    this->_closeDevice();
                    is_parsing_response      = false;
                    response_data_byte_count = 0;
                    respose_length           = 0;
                    response_index           = 0;
                    return;
                }

                // Getting response
                memset(_serial_in_buffer, 0, SERIAL_IN_BUFF_SIZE);

                std::size_t byte_count = read(Connector::get().getFd(this->_getOpenDeviceName()), _serial_in_buffer, SERIAL_IN_BUFF_SIZE);

                if(is_parsing_response) {
                    // Fallback: timeout if response isn't received completly within 200ms
                    auto elapsed_parsing_time =
                        s_chrono::duration_cast<s_chrono::milliseconds>(s_chrono::steady_clock::now() - response_paring_start);

                    if(elapsed_parsing_time.count() > RESPONSE_TIMEOUT) {
                        is_parsing_response = false;
                        response_index      = 0;
                        cwarn << "timeout receiving device response" << endl;
                    }
                }

                if (byte_count > 0) {
                    if(_serial_in_buffer[0] == MSG_START_CONDITION && !is_parsing_response) {
                        response_paring_start = s_chrono::steady_clock::now();
                        is_parsing_response   = true;
                        response_index        = 0;
                        device_response.clear();
                        is_parsing_response      = true;
                        response_data_byte_count = (int)(((std::uint16_t)(_serial_in_buffer[3]) << 8) | (std::uint16_t)_serial_in_buffer[2]);
                        respose_length           = response_data_byte_count + 5;
                    }

                    for (std::size_t i = 0; i < byte_count; i++) {
                        device_response.push_back(_serial_in_buffer[i]);
                        response_index++;
                    }

                    if(response_index == respose_length) {
                        atoms bytes_received;

                        is_parsing_response = false;
                        response_index      = 0;
                        _processDeviceResponds(device_response);
                    }
                }
            }
        }

        void _processDeviceResponds(const std::vector<unsigned char> received_bytes) {
            atoms response_message;
            int   breaktime_val;
            int   mabtime_val;
            int   refresh_val;
            char  firmware_version[8];
            char  serial_number_string[10];

            switch (received_bytes[1]) {
                case MSG_LABEL_GET_WIDGET_PARAMETRES:

                    breaktime_val = (int)((float)received_bytes[6] * 10.67f);
                    mabtime_val   = (int)((float)received_bytes[7] * 10.67f);
                    refresh_val   = (int)received_bytes[8];

                    snprintf(firmware_version, 8, "%d.%d", received_bytes[5], received_bytes[4] );
                    response_message.push_back(TO_OUTLET_DUMPOUT);
                    response_message.push_back("firmware");
                    response_message.push_back(std::string(firmware_version));
                    _enque_msg_to_max(response_message);
                    deliverer_to_max.delay(0);

                    response_message.clear();
                    response_message.push_back(TO_OUTLET_DUMPOUT);
                    response_message.push_back("breaktime");
                    response_message.push_back(breaktime_val);
                    _enque_msg_to_max(response_message);
                    deliverer_to_max.delay(0);

                    response_message.clear();
                    response_message.push_back(TO_OUTLET_DUMPOUT);
                    response_message.push_back("mabtime");
                    response_message.push_back(mabtime_val);
                    _enque_msg_to_max(response_message);
                    deliverer_to_max.delay(0);

                    response_message.clear();
                    response_message.push_back(TO_OUTLET_DUMPOUT);
                    response_message.push_back("refresh");
                    response_message.push_back(refresh_val);
                    _enque_msg_to_max(response_message);
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
                    _enque_msg_to_max(response_message);
                    deliverer_to_max.delay(0);
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
            this->_messages_to_device_queue.push(msg_send_dmx);
        }

    public:

        dmxusbpro_tilde(const atoms& args = {}) {
            memset(this->_dmx_universe, 0, 512);

            if (!args.empty()) {

                for(std::size_t i = 0; i < args.size(); i++) {
                    if (
                        args[i].type() != message_type::int_argument
                        || (int) args[i] > 512
                        || (int)args[i] < 1
                        ) {
                        this->_closeDevice();
                        error(OBJECT_MESSAGE_PREFIX + std::string(args[i]) + " bad argument: expected integer between 1 and 512");
                    }

                    int  dmx_channel = args[i];

                    if(std::find(_inlet_dmx_channel.begin(), _inlet_dmx_channel.end(), dmx_channel) != _inlet_dmx_channel.end()) {
                        this->_closeDevice();
                        error(std::string(OBJECT_MESSAGE_PREFIX) + "multile inlets assinged to the same DMX channel.");
                    }

                    auto dmx_inlet = std::make_unique<inlet<> >(this, "(signal) DMX channel " + std::string(args[i]));

                    _inlets.push_back(std::move(dmx_inlet));
                    _inlet_dmx_channel.push_back(dmx_channel);
                }
            }
        }

        ~dmxusbpro_tilde() {
            this->_closeDevice();
        }

        MIN_DESCRIPTION     { "Connect to the ENTTEC DMX USB Pro interface. Conrol DMX data with signals. <br/> The recommended firmware version is 1.44" };

        MIN_TAGS            { "utilities" };
        MIN_AUTHOR          { "Jan Mech" };
        MIN_RELATED         { "jam.dmxusbpro, serial" };

        argument<int> dmx_channel { this, "DMX_channels", "A list of DMX channel numbers. Each argument creates an signal inlet, to control the idicated channel." };

        inlet<> input_1    { this, "(signal) DMX master", "signal" };
        outlet<thread_check::scheduler, thread_action::fifo> output_1   { this, "DMX Output <startcode> <channel> <value>", "list" };
        outlet<> output_2   { this, "(int) State of Connection", "int" };
        outlet<> output_3   { this, "(anything) Connect to umenu" };
        outlet<> output_dumpout   { this, "dumpout", "anything"};

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
                    }
                }
                return {};
            }
        };

        attribute<int, threadsafe::no, limit::clamp, allow_repetitions::no> baudrate {
            this, "baudrate", 56700,
            title {"Device Baud Rate"},
            description{"Set the baud rate for communicating with the interface. Default: 56700"},
            range {9600, 256000},
            readonly {false},
            setter { MIN_FUNCTION {
                         if(Connector::get().isConnected(this->_getOpenDeviceName())) {
                             cerr << "baudrate has changed. closing the connection." << endl;
                             this->_closeDevice();
                         }

                         return args;
                     }
            }
        };

        attribute<bool, threadsafe::no, limit::none, allow_repetitions::no> keepsending {
            this, "keepsending", false,
            title { "Keep sending" },
            description { "If set to 0 (default), the device will stop sending DMX data when the connection is closed.<br />If set to 1 the device will continue to send the last received DMX data after the connection has been closed." }
        };

        attribute<int, threadsafe::yes, limit::clamp, allow_repetitions::no> push {
            this,
            "push",
            20,
            title { "Push Intermal (ms)" },
            description { "Minimum interval to push DMX value changes to the device in ms."},
            range { 10, 10000 }
        };

        attribute<bool, threadsafe::no, limit::none, allow_repetitions::no> verbose {
            this,
            "verbose",
            false,
            title { "Verbose" },
            description { "If set to 0 (default), only serial devices following the ENTTEC USB DMX Pro naming convention will be enabled in a umenu connected to the third outlet. <br /> If set to 1 all serial devices will be enabled and more information about the coinnection state will be printed to the Max console." }
        };

        message<threadsafe::yes> open {
            this, "open", "Open serial connection to a device. <p>Argument: portname[symbol]</p>",
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
                    try {
                        atom connected  = this->_connections.at(device_name);
                        int  conn_state = (int)connected;

                        if(conn_state == 1) {
                            cerr << "'" << device_name << "' already opened by another instance." << endl;
                            return {};
                        }
                    } catch (std::runtime_error& e) {
                        // the devices isn't opened and not registered
                        this->_connections[device_name] = 0;
                    }
                }

                if(verbose) {
                    cout << "opening " + device_name << endl;
                }

                if(!Connector::get().deviceExists(device_name)) {
                    cerr << "specified port not available" << endl;
                    return {};
                }

                this->_closeDevice();

                int         set_baudrate = baudrate;
                int         open_success = Connector::get().openSerialPort(device_name, set_baudrate);

                if(open_success == -1) {
                    cerr << "Error opening device" << endl;
                    return {};
                }

                if(open_success == -2) {
                    cerr << "'" << device_name << "' already opened by another instance." << endl;
                    return {};
                }

                this->_setOpenDeviceName(device_name);
                this->_connections[symbol(device_name)] = 1;
                atoms       connection_state;
                connection_state.push_back(TO_OUTLET_2);
                connection_state.push_back(1);
                _enque_msg_to_max(connection_state);
                deliverer_to_max.delay(0);

                this->_io_threads_continue = true;
                std::this_thread::sleep_for(s_chrono::milliseconds(10));

                this->_receive_thread = std::thread([this]()
            {
                atoms msg_to_console;

                if (verbose) {
                    msg_to_console.push_back(TO_MAX_CONSOLE);
                    msg_to_console.push_back("starting receive thread");
                    _enque_msg_to_max(msg_to_console);
                    deliverer_to_max.delay(0);
                }

                while(_io_threads_continue) {
                    _receiveThreadTask();
                }

                if (verbose) {
                    msg_to_console.clear();
                    msg_to_console.push_back(TO_MAX_CONSOLE);
                    msg_to_console.push_back("stopping receive thread");
                    _enque_msg_to_max(msg_to_console);
                    deliverer_to_max.delay(0);
                }
            });
                this->_receive_thread.detach();

                this->_send_thread = std::thread([this]()
            {
                atoms msg_to_console;

                if (verbose) {
                    msg_to_console.push_back(TO_MAX_CONSOLE);
                    msg_to_console.push_back("starting send thread");
                    _enque_msg_to_max(msg_to_console);
                    deliverer_to_max.delay(0);
                }

                while(_io_threads_continue) {
                    _sendThreadTask();
                }

                if (verbose) {
                    if (verbose) {
                        msg_to_console.clear();
                        msg_to_console.push_back(TO_MAX_CONSOLE);
                        msg_to_console.push_back("stopping send thread");
                        _enque_msg_to_max(msg_to_console);
                        deliverer_to_max.delay(0);
                    }
                }
            });
                this->_send_thread.detach();
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

                std::vector<std::string> device_names = Connector::get().getDeviceNames(verbose, true);

                for (auto& device_name : device_names) {
                    atoms device_list;
                    device_list.push_back("append");
                    device_list.push_back(device_name);
                    output_3.send(device_list);
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

                if(!Connector::get().isConnected(this->_getOpenDeviceName())) {
                    if(verbose) {
                        cerr << "Can't get DMX parameters, not connected." << endl;
                    }

                    return {};
                }

                this->_messages_to_device_queue.push(std::vector<unsigned char> {
                MSG_START_CONDITION,
                MSG_LABEL_GET_WIDGET_PARAMETRES,
                0x00, 0x00,
                MSG_END_CONDITION
            });

                return {};
            }
        };

        message<threadsafe::yes> deviceserial {
            this, "deviceserial", "Get the devices serial number.",
            MIN_FUNCTION {
                if(args.size() > 0) {
                    cwarn << "extra argument for message 'getserial'" << endl;
                }

                if(!Connector::get().isConnected(this->_getOpenDeviceName())) {
                    if(verbose) {
                        cerr << "Can't get serial number, not connected." << endl;
                    }

                    return {};
                }

                this->_messages_to_device_queue.push(std::vector<unsigned char> {
                MSG_START_CONDITION,
                MSG_LABEL_GET_WIDGET_SERIAL_NUMBER,
                0x00, 0x00,
                MSG_END_CONDITION});
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

                this->_closeDevice();
                return{};
            }
        };

        void operator ()(audio_bundle input, audio_bundle output) {
            static auto                         last_run = s_chrono::steady_clock::now();
            static std::map<int, unsigned char> prev_dmx_vals;
            std::map<int, unsigned char>        current_dmx_vals;
            auto                                now           = s_chrono::steady_clock::now();
            auto                                elaped        = s_chrono::duration_cast<s_chrono::milliseconds>(now - last_run);
            int                                 push_interval = push;

            if(elaped.count() >= push_interval) {
                last_run = s_chrono::steady_clock::now();

                auto   in_master = input.samples(0);
                double master    = std::min(1., std::max(0., in_master[0]));

                for (std::size_t inlet_index = 0; inlet_index < _inlets.size(); inlet_index++) {
                    auto          in             = input.samples(inlet_index + 1); // first inlet is the master concrol
                    double        channel_sample = std::min(1., std::max(0., in[0]));
                    unsigned char dmx_value      = round(channel_sample * master * 255.);

                    current_dmx_vals.emplace(this->_inlet_dmx_channel[inlet_index], dmx_value);
                }

                if(current_dmx_vals != prev_dmx_vals) {
                    auto it = current_dmx_vals.begin();

                    while (it != current_dmx_vals.end()) {
                        this->_dmx_universe[it->first - 1] = it->second;
                        it++;
                    }
                    prev_dmx_vals = current_dmx_vals;
                    this->_enqueMsgSendDmxPpacket(this->_dmx_universe);
                }
            }
        }
};


MIN_EXTERNAL(dmxusbpro_tilde);
