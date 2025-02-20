#include "jam.dmxusbpro.dmx_device.hpp"


std::vector<std::string> & Connector::getDevicePaths() {
    return this->m_device_paths;
}


std::vector<std::string> Connector::getDeviceNames(bool verbose, bool reload) {

    std::vector<std::string> device_names;

    if(this->m_device_paths.size() == 0 || reload) {
        this->loadDevices();
    }

    for (auto& device_path : this->m_device_paths) {
        size_t           start_pos = device_path.find("/dev/");

        if (std::string::npos == start_pos) {
            continue;
        }

        std::string      device_name = device_path.substr(8);
        const std::regex enttec_rexex("^usbserial-EN[0-9]+");

        if (!std::regex_match(device_name, enttec_rexex) && !verbose) {
            device_name = "(" + device_name + ")";
        }

        device_names.push_back(device_name);
    }

    return device_names;
}

void Connector::loadDevices() {
    std::vector<std::string> serial_devices;
    kern_return_t            kernResult;
    io_iterator_t            serialPortIterator;

    this->m_device_paths.clear();
    kernResult = this->_findModems(&serialPortIterator);

    if (KERN_SUCCESS != kernResult) {
        goto exit;
    }

    kernResult = this->_getModemPaths(serialPortIterator, serial_devices);

    if (KERN_SUCCESS != kernResult) {
        goto exit;
    }

 exit:
    this->m_device_paths = serial_devices;
}

kern_return_t Connector::_findModems(io_iterator_t *matchingServices) {

    kern_return_t          kernResult = KERN_FAILURE;
    CFMutableDictionaryRef classesToMatch;

    classesToMatch = IOServiceMatching(kIOSerialBSDServiceValue);

    if (classesToMatch == NULL) {
        goto exit;
    } else {
        // Look for devices that claim to be modems.
        CFDictionarySetValue(
            classesToMatch,
            CFSTR(kIOSerialBSDTypeKey),
            CFSTR(kIOSerialBSDAllTypes)
            );
    }

    // Get an iterator across all matching devices.
    kernResult = IOServiceGetMatchingServices(kIOMasterPortDefault, classesToMatch, matchingServices);

    if (KERN_SUCCESS != kernResult) {
        goto exit;
    }

 exit:
    return kernResult;
}

kern_return_t Connector::_getModemPaths(io_iterator_t serialPortIterator,std::vector<std::string>& path_map) {
    io_object_t   modemService;
    kern_return_t kernResult = KERN_FAILURE;
    char          bsdPath[1024];

    *bsdPath = '\0';

    while ((modemService = IOIteratorNext(serialPortIterator))) {
        CFTypeRef bsdPathAsCFString;

        // Get the callout device's path (/dev/cu.xxxxx). The¬ callout device should almost always be
        // used: the dialin device (/dev/options.xxxxx) would be used when monitoring a serial port for
        // incoming calls, e.g. a fax listener.

        bsdPathAsCFString = IORegistryEntryCreateCFProperty(modemService,
                                                            CFSTR(kIOCalloutDeviceKey),
                                                            kCFAllocatorDefault,
                                                            0);

        if (bsdPathAsCFString) {
            Boolean           result;

            // Convert the path from a CFString to a C (NUL-terminated) string for use
            // with the POSIX open() call.
            result = CFStringGetCString((CFStringRef)bsdPathAsCFString,
                                        bsdPath,
                                        1024,
                                        kCFStringEncodingUTF8);

            const std::string temp_path(bsdPath);

            path_map.push_back(temp_path);
            CFRelease(bsdPathAsCFString);

            if (result && KERN_SUCCESS != kernResult) {
                kernResult = KERN_SUCCESS;
            }
        }

        // Release the io_service_t now that we are done with it.
        (void)IOObjectRelease(modemService);
    }

    return kernResult;
}

int  Connector::openSerialPort(const std::string port_name, const speed_t baud_rate) {

    // check if device is already opened.
    if(this->isConnected(port_name)) {
        return -2;
    }

    struct termios options;

    int            fd = -1;

    std::string    full_device_path = "/dev/cu." + port_name;
    const char*    c_port_name      = full_device_path.c_str();

    fd = open(c_port_name, O_RDWR);

    if (fd < 0) {
        goto fail;
    }

    ioctl(fd, TIOCGETA, &options);

    options.c_cflag &= ~PARENB;           // Clear parity bit, disabling parity (most common)
    options.c_cflag &= ~CSTOPB;           // Clear stop field, only one stop bit used in communication (most common)
    options.c_cflag &= ~CSIZE;            // Clear all bits that set the data size
    options.c_cflag |= CS8;               // 8 bits per byte (most common)
    options.c_cflag &= ~CRTSCTS;          // Disable RTS/CTS hardware flow control (most common)
    options.c_cflag |= CREAD | CLOCAL;    // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    options.c_lflag &= ~ICANON;
    options.c_lflag &= ~ECHO;             // Disable echo
    options.c_lflag &= ~ECHOE;            // Disable erasure
    options.c_lflag &= ~ECHONL;           // Disable new-line echo
    options.c_lflag &= ~ISIG;             // Disable interpretation of INTR, QUIT and SUSP
    options.c_iflag &= ~(IXON | IXOFF | IXANY);    // Turn off s/w flow ctrl
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);    // Disable any special handling of received bytes

    options.c_oflag &= ~OPOST;    // Prevent special interpretation of output bytes (e.g. newline chars)
    options.c_oflag &= ~ONLCR;    // Prevent conversion of newline to carriage return/line feed
    // options.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // options.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    options.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    options.c_cc[VMIN]  = 0;

    ioctl(fd, TIOCSETA, &options);

    // The IOSSIOSPEED ioctl can be used to set arbitrary baud rates other than
    // those specified by POSIX. The driver for the underlying serial hardware
    // ultimately determines which baud rates can be used. This ioctl sets both
    // the input and output speed.


    if (ioctl(fd, IOSSIOSPEED, &baud_rate) == -1) {
        printf("[WARN] ioctl(..., IOSSIOSPEED, %lu).\n", baud_rate);
        goto fail;
    }

    // Check that speed is properly modified
    if (tcgetattr(fd, &options) == -1) {
        printf("[WARN] _modifyAttributes: tcgetattr failed\n");
        goto fail;
    }

    if (cfgetispeed(&options) != baud_rate ||
        cfgetospeed(&options) != baud_rate) {
        printf("[WARN] _modifyAttributes: cfsetspeed failed, %lu, %lu.\n",
               baud_rate,
               cfgetispeed(&options));
        goto fail;
    }

    // If successfull opened store termios options and file descriptor in maps
    this->_addConnection(port_name, options, fd);
    return fd;

 fail:

    // Opening port faild: remove termios options and file descriptor from maps
    this->_removeConenction(port_name);

    return -1;
}

int  Connector::closeSerialPort(std::string port_name) {
    int close_success = 0;
    int fd            = this->getFd(port_name);


    if(fd != -1) {
        flock(fd, LOCK_UN | LOCK_NB); // unlock file
        close_success = close(fd);

        if(close_success != -1) {
            this->_removeConenction(port_name);
        }
    }

    return close_success;
}

int  Connector::getFd(std::string port_name) {
    int fd = -1;

    if(this->isConnected(port_name)) {
        fd = this->_connections[port_name].fid;
    }

    return fd;
}

bool Connector::isConnected(std::string port_name) {
    this->connections_lock.lock();

    bool is_connected = this->_connections.find(port_name) != this->_connections.end();

    this->connections_lock.unlock();
    return is_connected;
}

bool Connector::deviceExists(std::string port_name) {
    std::vector<std::string> current_devices = this->getDeviceNames(true, true);

    return std::find(
        current_devices.begin(),
        current_devices.end(),
        port_name
        ) != current_devices.end();
}

int  Connector::connectionState(std::string port_name) {
    int     fd    = this->getFd(port_name);
    termios options_stored;
    termios options_device;

    this->_getSerialOptions(port_name, options_stored);

    if (tcgetattr(fd, &options_device) < 0 ) {
        return ConnectionState::MISSING;
    }

    // Check if someone else has modified the connection config
    if(
        options_stored.c_cflag != options_device.c_cflag
        || options_stored.c_iflag !=options_device.c_iflag
        || options_stored.c_lflag !=options_device.c_lflag
        || options_stored.c_cc[VTIME] !=options_device.c_cc[VTIME]
        || options_stored.c_cc[VMIN] !=options_device.c_cc[VMIN]
        || options_stored.c_ispeed !=options_device.c_ispeed
        || options_stored.c_ospeed !=options_device.c_ospeed
        ) {
            return ConnectionState::MODOFIED;
    }

    return ConnectionState::OK;
}

void Connector::_getSerialOptions(std::string port_name, termios &serial_option) {
    if(this->isConnected(port_name)) {
        serial_option = this->_connections[port_name].options;
    }
}

void Connector::_addConnection(const std::string port_name, const termios options, const int file_descriptor) {
    connection_t connection {
        options,
        file_descriptor
    };

    this->connections_lock.lock();
    this->_connections[port_name] = connection;
    this->connections_lock.unlock();
}

void Connector::_removeConenction(const std::string port_name) {
    this->connections_lock.lock();

    if(this->_connections.find(port_name) != this->_connections.end()) {
        this->_connections.erase(port_name);
    }

    this->connections_lock.unlock();
}
