#pragma once

#include <algorithm>
#include <cstdint>
#include <CoreFoundation/CFString.h>
#include <CoreFoundation/CoreFoundation.h>
#include <errno.h> // Error integer and strerror() function
#include <fcntl.h>
#include <iostream>
#include <IOKit/IOBSD.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/ioss.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <mutex>
#include <regex>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <vector>


// Common definitions for jam.dmxusbpro and jam.dmxusbpro~
#define SERIAL_IN_BUFF_SIZE                  700
#define MSG_START_CONDITION                  0x7E
#define MSG_END_CONDITION                    0xE7
#define MSG_LABEL_GET_WIDGET_PARAMETRES      0x03
#define MSG_LABEL_SET_WIDGET_PARAMETRES      0x04
#define MSG_LABEL_RECEIVED_DMX_PACKET        0x05
#define MSG_LABEL_SEND_DMX_PACKET            0x06
#define MSG_LABEL_RECEIVE_DMX                0x08
#define MSG_LABEL_RECEIVED_DMX_PACKET_CHANGE 0X09
#define MSG_LABEL_GET_WIDGET_SERIAL_NUMBER   0x0A
#define TO_OUTLET_1                          0x00
#define TO_OUTLET_2                          0x01
#define TO_OUTLET_3                          0x02
#define TO_OUTLET_DUMPOUT                    0x03
#define TO_MAX_CONSOLE_WARN                  0xFE
#define TO_MAX_CONSOLE                       0xFF
#define RESPONSE_TIMEOUT                     250

class Connector {

    typedef struct {
        termios options;
        int fid;
    } connection_t;

    typedef std::unordered_map<std::string, connection_t> connection_map_t;

    public:

        enum ConnectionState {
            OK,
            MISSING,
            MODOFIED
        };

        Connector(const Connector&) = delete;

        static Connector & get() {
            static Connector instance;

            return instance;
        }

        std::mutex       connections_lock;

        void loadDevices();
        bool deviceExists(std::string port_name);
        std::vector<std::string>&getDevicePaths();
        std::vector<std::string> getDeviceNames(bool verbose, bool reload = false);
        int openSerialPort(std::string port_name, speed_t baud_rate);
        int closeSerialPort(std::string port_name);
        int getFd(std::string port_name);
        bool isConnected(std::string port_name);
        int connectionState(std::string port_name);

    private:
        Connector() {};

        std::vector<std::string> m_device_paths;
        connection_map_t _connections;
        kern_return_t _findModems(io_iterator_t *matchingServices);
        kern_return_t _getModemPaths(io_iterator_t serialPortIterator, std::vector<std::string>& path_map);
        void _addConnection(std::string port_name, termios options, int file_descriptor);
        void _removeConenction(std::string port_name);
        void _getSerialOptions(std::string port_name, termios &serial_options);
};
