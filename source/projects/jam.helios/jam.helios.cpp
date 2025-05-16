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
#include "../jam.helper/attribute_args_helper.hpp"
#include "../jam.helios.connector/jam.helios.connector.hpp"

#define POINTS_PER_FRAME 1000
#define X_Y_MAX 65500
#define X_Y_MIN 35


using namespace c74::min;
using HeliosConnector = jam::helios::Connector;
using fvec = std::vector<number>;


typedef struct LaserPoint {
    uint16_t x = 32767; // 65535 (0xFFFF)  / 2
    uint16_t y = 32767; // 65535 (0xFFFF)  / 2
    bool blanking = false;
} laser_point_t;

typedef struct CoordPoint {
    number x = 0.;
    number y = 0.;
} coord_point_t;

using lpvec = std::vector<laser_point_t>;


class helios : public object<helios>
{
    
protected:
    
    HeliosPointHighRes* frame_1 = nullptr;
    HeliosPointHighRes* frame_2 = nullptr;
    HeliosPointHighRes* frame_play = nullptr;
    HeliosPointHighRes* frame_edit = nullptr;
    
    lpvec _frame_points;
        
    uint16_t _color[3] = {0xFFFF, 0xFFFF,0xFFFF};
    
    uint _instance_id = 0;                  // Unique ID for each object instance.
    
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
    
    int _attached_device = -1;
    
    std::thread _projector_thread;
    
    bool _projector_in_running = false;
    
    fifo<queued_message_t> _to_max_queue_2{ 1000 };
    
    std::mutex _enqueue_msg_lock;
    
    c74::max::t_object * _manager = nullptr;
    
    HeliosConnector * _connector = nullptr;
    
    HeliosConnector * _getConnector() {
        if(this->_connector == nullptr) {
            this->_connector = (HeliosConnector *)typedmess(this->_manager,symbol("get_connector"),0,0L);
        }
        return this->_connector;
    }
    
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
    
    std::string heliosErrorToString(int error) {
        
        switch (error) {
            case HELIOS_SUCCESS:
                return "HELIOS_SUCCESS";
            case HELIOS_ERROR_NOT_INITIALIZED:
                return "HELIOS_ERROR_NOT_INITIALIZED";
            case HELIOS_ERROR_INVALID_DEVNUM:
                return "HELIOS_ERROR_INVALID_DEVNUM";
            case HELIOS_ERROR_NULL_POINTS:
                return "HELIOS_ERROR_NULL_POINTS";
            case HELIOS_ERROR_TOO_MANY_POINTS:
                return "HELIOS_ERROR_TOO_MANY_POINTS";
            case HELIOS_ERROR_PPS_TOO_HIGH:
                return "HELIOS_ERROR_PPS_TOO_HIGH";
            case HELIOS_ERROR_PPS_TOO_LOW:
                return "HELIOS_ERROR_PPS_TOO_LOW";
            case HELIOS_ERROR_DEVICE_CLOSED:
                return "HELIOS_ERROR_PPS_TOO_LOW";
            case HELIOS_ERROR_DEVICE_FRAME_READY:
                return "HELIOS_ERROR_DEVICE_FRAME_READY";
            case HELIOS_ERROR_DEVICE_SEND_CONTROL:
                return "HELIOS_ERROR_DEVICE_SEND_CONTROL";
            case HELIOS_ERROR_DEVICE_RESULT:
                return "HELIOS_ERROR_DEVICE_RESULT";
            case HELIOS_ERROR_DEVICE_NULL_BUFFER:
                return "HELIOS_ERROR_DEVICE_NULL_BUFFER";
            case HELIOS_ERROR_DEVICE_SIGNAL_TOO_LONG:
                return "HELIOS_ERROR_DEVICE_SIGNAL_TOO_LONG";
            case HELIOS_ERROR_NOT_SUPPORTED:
                return "HELIOS_ERROR_NOT_SUPPORTED";
            case HELIOS_ERROR_NETWORK:
                return "HELIOS_ERROR_NETWORK";
            case HELIOS_ERROR_LIBUSB_BASE:
                return "HELIOS_ERROR_LIBUSB_BASE";
            default:
                return "UNKNOWN ERROR";
        }
    }
    
    uint getInstanceId() {
        return this->_instance_id;
    }
    
    void _fillFrame(HeliosPointHighRes * frame, lpvec lps) {
        std::mutex lock;
        lock.lock();
        for (int i = 0; i < POINTS_PER_FRAME; i++) {
            frame[i].x = lps[i].x;
            frame[i].y = lps[i].y;
            frame[i].r = lps[i].blanking ? 0 : this->_color[0];
            frame[i].g = lps[i].blanking ? 0 : this->_color[1];
            frame[i].b = lps[i].blanking ? 0 : this->_color[2];
            
        }
        lock.unlock();
    }
    
    void _drawFrame() {
        std::mutex lock;
        lock.lock();
        HeliosPointHighRes* old_frame_play = this->frame_play;
        this->frame_play = this->frame_edit;
        this->frame_edit = old_frame_play;
        lock.unlock();
    }
    
    number _map(number x, number in_min, number in_max, number out_min, number out_max)
    {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
    number _toPrecision(number value, uint precision = 3) {
        value = (int)(value * (10 * precision));
        return (number)value / (10 * precision);
    }
    
    laser_point_t _toLaserPoint(coord_point_t p) {
        laser_point_t lp;
        // from -1/1 to 0/0xFFFF (65535)
        number x_mapped = this->_map(p.x, -1., 1., 0., 65535.);
        number y_mapped = this->_map(p.y, -1., 1.,  0., 65535.);
        lp.x = (uint16_t)x_mapped;
        lp.y = (uint16_t)y_mapped;
        return lp;
        
    };
    
    
    lpvec _makeEllipsePoints(
        coord_point_t c,
        coord_point_t r,
        const number theta_start = 0,
        const number theta_end = 360,
        int segments = 1000
    ) {
        lpvec points;
        if(theta_start == theta_end) {
            return points;
        }
        number rad_start = theta_start * (PI / 180.);
        number rad_end = theta_end * (PI / 180.);
        number rad_range = rad_end - rad_start;
        
        
        for (int i = 0; i < segments; ++i) {
            coord_point_t cp;
           
            number angle = rad_start + (rad_range * i / segments);
            // number angle = rad_start + (number)(rad_range * (number)i / (number)segments);
            cp.x = c.x + r.x * std::cos(angle);
            cp.y = c.y + r.y * std::sin(angle);
            
            cp.x = std::clamp(cp.x, -1., 1.);
            cp.y = std::clamp(cp.y, -1., 1.);
            
            laser_point_t lp = this->_toLaserPoint(cp);
            bool visible = lp.x >= X_Y_MIN && lp.x <= X_Y_MAX && lp.y >= X_Y_MIN && lp.y <= X_Y_MAX;
            lp.blanking = !visible;
            points.push_back(lp);
        }
        points[segments - 1] = points[0];
        return points;
        
    }
    
    lpvec _makeDotPoints(number x, number y) {
        lpvec points;
        number x_coord_prev = 0.;
        for(int i = 0; i < POINTS_PER_FRAME; i++) {
            coord_point_t cp;
            cp.y = y;
            number offset = (2. / (number)(POINTS_PER_FRAME) * (number)i);
            number x_coord = -1. + offset;
            cp.x = x_coord;
            bool blanking = !(x >= x_coord_prev && x <= x_coord);
            laser_point_t lp = this->_toLaserPoint(cp);
            lp.blanking = blanking;
            x_coord_prev = x_coord;
            points.push_back(lp);
        }
        return points;
    }
 
    
public:
    helios(const atoms& args = {}) {
        if (!dummy()) {
            this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.helios.manager"), 0, NULL);
            this->_connector = (HeliosConnector *)typedmess(this->_manager,symbol("get_connector"),0,0L);
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            srand((unsigned int)ts.tv_nsec);
            this->_instance_id = rand();
            
            this->frame_1 =  new HeliosPointHighRes[POINTS_PER_FRAME];
            this->frame_2 =  new HeliosPointHighRes[POINTS_PER_FRAME];
            laser_point_t lp{0, 0};
            lpvec empty_frame{POINTS_PER_FRAME, lp};
            this->_frame_points = empty_frame;
            this->_fillFrame(this->frame_1, empty_frame);
            this->_fillFrame(this->frame_2, empty_frame);
            this->frame_play = this->frame_1;
            this->frame_edit = this->frame_2;
        }
    }
    
    ~helios() {
        if (!dummy()) {
            close();
//            delete[] this->frame_1;
//            delete[] this->frame_2;
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

    attribute<int, threadsafe::no, limit::clamp> samplerate {
        this,
        "samplerate",
        30000,
        title{ "Samplerate" },
        description{ "Points per second send to the laser projector.<br /><b>Note</b>:It is recommended to keep sampling rate at 30000 or below, as higher values can cause problems in certain devices like LaserCube Wifi." },
        range{ 1000, 100000 },
    };
    
    attribute<int, threadsafe::no, limit::clamp> fps {
        this,
        "fps",
        60,
        title{ "Frame Per Second" },
        description{ "" },
        range{ 1, 200 },
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
    
    attribute<fvec> lasercolor {
        this, "lasercolor", { 1., 1, 1., 1.},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            cleaned_args[3] = 1.;
            this->_color[0] = static_cast<uint16_t>((number)cleaned_args[0] * (number)0xFFFF);
            this->_color[1] = static_cast<uint16_t>((number)cleaned_args[1] * (number)0xFFFF);
            this->_color[2] = static_cast<uint16_t>((number)cleaned_args[2] * (number)0xFFFF);
            if(this->initialized()) {
                this->_fillFrame(this->frame_edit, this->_frame_points);
                this->_drawFrame();
            }
            return cleaned_args;
        }},
        title {"Laser Color"},
        description {"Laser Color"},
        style {c74::min::style::color},
    };
    

    message<> open {
        this, "open", "Open connetion to a Helios DAC",
        MIN_FUNCTION{
            if (args.size() == 0){
                return {};
            }
            if(args[0].type() != message_type::int_argument && args[0].type() != message_type::float_argument) {
                return {};
            }
            int device_index = args[0];
            if(device_index < 1) {
                return {};
            }
            
            device_index--;
            
            auto result = this->_getConnector()->attachDevice(device_index, this->getInstanceId());
            
            bool connection_state = false;
            switch(result) {
                case jam::helios::DeviceState::ATTACH_SUCCESS:
                    connection_state = true;
                    this->_attached_device = device_index;
                    break;
                case jam::helios::DeviceState::NOTFOUND:
                    cwarn << "device index " << device_index << " out of range" << endl;
                    break;
                case jam::helios::DeviceState::ATTACH_ERROR_ALREADY_ATTACHED:
                    cwarn << "device " << device_index << " already opened by another instance" << endl;
                    break;
                default:
                    break;
            }

            
            queued_message_t msg;
            atoms msg_atoms;
            msg_atoms.push_back(connection_state);
            msg.set(&outlet_connected, msg_atoms);
            msg.send(this);
            return {};
        }
    };
    
    message<> close {
        this, "close", "Close connetion to Helios DAC",
        MIN_FUNCTION {
            if(this->_attached_device > -1) {
                this->_getConnector()->detachDevice(this->getInstanceId());
                this->_attached_device = -1;
                queued_message_t msg;
                atoms msg_atoms;
                msg_atoms.push_back(0);
                msg.set(&outlet_connected, msg_atoms);
                msg.send(this);
            }
            return {};
        }
    };
    
    message<> shutter {
        this, "shutter", "Open/Close the shutter. <p><b>Argument:</b><br /> shutter_state [int]</p>",
        MIN_FUNCTION{
//            if (args.size() < 1){
//                cwarn << "missing argument for message shutter" << endl;
//                return {};
//            }
//            if (args.size() > 1) {
//                cwarn << "extra argument for message shutter" << endl;
//            }
//            int shutter_state_int = (int)args[0];
//            bool shutter_state = shutter_state_int = !0;
//            this->_deviceManager.setShutter(this->maxobj(), shutter_state);
            return {};
        }
    };
    
    message<threadsafe::no> dot {
        this, "dot", "Project a dot at position x/y",
        MIN_FUNCTION {
            if(args.size() < 2) {
                return {};
            }
            number x = std::clamp((number)args[0], -1., 1.);
            number y = std::clamp((number)args[1], -1., 1.) * -1.;
            this->_frame_points = this->_makeDotPoints(x, y);
            this->_fillFrame(this->frame_edit, this->_frame_points);
            this->_drawFrame();
            return {};
        }
    };
    
    message<threadsafe::no> circle {
        this, "circle", "Project a circle at position x/y with radus r",
        MIN_FUNCTION {
            if(args.size() < 2) {
                return {};
            }
            number r = 0.3;
            if (args.size() >= 3) {
                r = std::clamp((number)args[2], 0., 1.);
            }
            number x = std::clamp((number)args[0], -1., 1.);
            number y = std::clamp((number)args[1], -1., 1.) * -1.;
            coord_point_t center = {x, y};
            coord_point_t  radius = {r, r};
            
            this->_frame_points = this->_makeEllipsePoints(center, radius);
            this->_fillFrame(this->frame_edit, this->_frame_points);
            this->_drawFrame();
            return {};
        }
    };
    
    
    
    message<threadsafe::no> integer {
        this, "int", "Start scanning",
        MIN_FUNCTION {
            if(this->_attached_device < 0) {
                return {};
            }
            bool run = static_cast<int>(args[0]) != 0;
            if(!run) {
                this->_projector_in_running = false;
                this->_getConnector()->getDac()->SetShutter(this->_attached_device, false);
                return {};
            }
            if(this->_projector_in_running) {
                return {};
            }
            this->_projector_in_running = true;
            
            this->_projector_thread = std::thread([this]() {
                HeliosDac * helios = this->_getConnector()->getDac();
                while (this->_projector_in_running) {
                    int status = helios->GetStatus(this->_attached_device);
                    if (status == 1) {
                        int result = helios->WriteFrameHighResolution(this->_attached_device, (int)samplerate, HELIOS_FLAGS_DEFAULT, this->frame_play, POINTS_PER_FRAME);
                        if(result != HELIOS_SUCCESS) {
                            cerr << this->heliosErrorToString(result) << endl;
                        }
                        std::this_thread::sleep_for (std::chrono::milliseconds(1000 / (int)fps));
                    }
                }
//                delete[] frame;
            });
            this->_projector_thread.detach();
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

MIN_EXTERNAL(helios);
