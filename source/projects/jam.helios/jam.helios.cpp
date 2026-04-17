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
#include  "../jam.ilda_common/ilda_definitions.hpp"
#include "../jam.ilda_common/ilda_colors.hpp"
#include "../jam.ilda.manager/jam.ilda.manager.hpp"
#include "../jam.ilda_common/ilda_frame.hpp"
#include "../jam.ilda_common/ilda_header.hpp"
#include "../jam.ilda_common/ilda_data_record.hpp"
#include "../jam.ilda_common/ilda_colors.hpp"
#include "InterfaceHeliosListener.hpp"

#define POINTS_PER_FRAME 1000
#define X_Y_MAX 65500
#define X_Y_MIN 35


using namespace c74::min;
using namespace jam::helios;
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


class helios : public object<helios>, public InterfaceHeliosListener
{
    
protected:
    
    HeliosPointHighRes* _frame_1    = nullptr;
    HeliosPointHighRes* _frame_2    = nullptr;
    HeliosPointHighRes* _play_frame = nullptr;
    HeliosPointHighRes* _edit_frame = nullptr;
    
    std::mutex _frame_1_lock;
    std::mutex _frame_2_lock;
    std::mutex _play_frame_lock;
    std::mutex _edit_frame_lock;
    
    std::vector<jam::ilda::IldaFrame>                _ilda_frames;
//    std::vector<std::unique_ptr<HeliosPointHighRes[]>> _laser_frames;
    std::vector<std::vector<HeliosPointHighRes>> _laser_frames;
    
    lpvec _frame_points;
    std::mutex _frame_points_lock;
    
    coord_point_t _scaling {1., 1.};
    
    number _rotation_angle = 0.;
    
    coord_point_t _rotation_center = {0., 0.};
        
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
    
    c74::max::t_object *_ilda_manager;              // Pointer to global jam.ilda.manager object
                                                    // (stores data to be accasibele by other jam.ilda.* object)
    t_jam_im * _ilda_manager_struct_ptr = NULL;     // Pointer to max-object struct of the jam.ilda.manager object
    
    t_jam_im * _getIldaManagerStructPointer() {
        if(this->_ilda_manager_struct_ptr == NULL) {
            this->_ilda_manager_struct_ptr = (t_jam_im *)typedmess(this->_ilda_manager,symbol("get_struct"),0,0L);
        }
        return this->_ilda_manager_struct_ptr;
    }
    
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
    
    lpvec _rotateAndScale(const lpvec & lps) {
        lpvec rotated;
        number cos_a      = std::cos(this->_rotation_angle);
        number sin_a      = std::sin(this->_rotation_angle);
        for(auto lp: lps) {
            coord_point_t coord_point = this->_toCoordPoint(lp);
            
                // apply rotation
                // calculate delta x/y - ajust for rotation anchor
            coord_point_t delta = {coord_point.x - this->_rotation_center.x, coord_point.y - this->_rotation_center.y};
       
            coord_point.x = (delta.x * cos_a) - (delta.y * sin_a) + this->_rotation_center.x;
            coord_point.y = (delta.x * sin_a) + (delta.y * cos_a) + this->_rotation_center.y;
            
                //apply scaling
            coord_point.x = coord_point.x * this->_scaling.x;
            coord_point.y = coord_point.y * this->_scaling.y;
            
            coord_point.x = std::clamp(coord_point.x, -1., 1.);
            coord_point.y = std::clamp(coord_point.y, -1., 1.);
            
            laser_point_t rp = this->_toLaserPoint(coord_point);
            rp.blanking = lp.blanking;
            if(!this->_laserPointVisible(rp)) {
                rp.blanking = true;
            }
            rotated.push_back(rp);
        }
        return rotated;
    }
    
    void _fillFrame(HeliosPointHighRes * frame, lpvec lps) {
        lpvec processed = this->_rotateAndScale(lps);
       
        for (int i = 0; i < POINTS_PER_FRAME; i++) {
            frame[i].x = processed[i].x;
            frame[i].y = processed[i].y;
            frame[i].r = processed[i].blanking ? 0 : this->_color[0];
            frame[i].g = processed[i].blanking ? 0 : this->_color[1];
            frame[i].b = processed[i].blanking ? 0 : this->_color[2];
        }
    }
    
    void _drawFrame() {
        std::scoped_lock lock(_play_frame_lock, _edit_frame_lock);
        HeliosPointHighRes* old_frame_play = this->_play_frame;
        this->_play_frame = this->_edit_frame;
        this->_edit_frame = old_frame_play;
        
    }
    
    void _setFramePoints(lpvec frame_points) {
        std::lock_guard lock(_frame_points_lock);
        this->_frame_points = frame_points;
    }
    
//    number _map(number x, number in_min, number in_max, number out_min, number out_max) {
//      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//    }
    
    template <typename T> T _map(T x, T in_min, T in_max, T out_min, T out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
    number _toPrecision(number value, uint precision = 3) {
        value = (int)(value * (10 * precision));
        return (number)value / (10 * precision);
    }
    
    bool _laserPointVisible(laser_point_t &lp) {
        
        return lp.x >= X_Y_MIN && lp.x <= X_Y_MAX && lp.y >= X_Y_MIN && lp.y <= X_Y_MAX;
    }
    
    laser_point_t _toLaserPoint(coord_point_t p, bool set_blanking = true) {
        laser_point_t lp;
        // from -1/1 to 0/0xFFFF (65535)
        number x_mapped = this->_map(p.x, -1., 1., 0., 65535.);
        number y_mapped = this->_map(p.y, -1., 1.,  0., 65535.);
        lp.x = (uint16_t)x_mapped;
        lp.y = (uint16_t)y_mapped;
        if(set_blanking) {
            lp.blanking = !this->_laserPointVisible(lp);
        }
        return lp;
    };
    
    coord_point_t _toCoordPoint(const laser_point_t &lp) {
        coord_point_t cp;
    
        cp.x = this->_map((number)lp.x, 0., 65535., -1., 1.);
        cp.y = this->_map((number)lp.y, 0., 65535., -1., 1.);
        return cp;;
    }
    
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
            laser_point_t lp = this->_toLaserPoint(cp, false);
            lp.blanking = blanking;
            x_coord_prev = x_coord;
            points.push_back(lp);
        }
        return points;
    }
    
    lpvec _makeLinePoints(
          const coord_point_t& p1,
          const coord_point_t& p2
          ) {
        lpvec points;
        number delta_x = p2.x - p1.x;
        number delta_y = p2.y - p1.y;
        for (int i = 0; i < POINTS_PER_FRAME; ++i) {
            number t = static_cast<number>(i) / POINTS_PER_FRAME;
            number x = p1.x + t * delta_x;
            number y = p1.y + t * delta_y;
            coord_point_t cp{std::clamp(x, -1., 1.), std::clamp(y, -1., 1.)};
            laser_point_t lp = this->_toLaserPoint(cp);
            points.push_back(lp);
        }
        for(int i = 1; i < 10; i++) {
            points[POINTS_PER_FRAME - i] = points[0];
            points[POINTS_PER_FRAME - i].blanking = true;
        }
        
        return points;
    }
    
    void _parseFrames2DToTrueColor(std::vector<jam::ilda::IldaFrame> &frames) {
        jam::ilda::Colors *col = new jam::ilda::Colors();
        
        for(size_t i = 0; i < frames.size(); i++) {
            jam::ilda::IldaHeader h = frames[i].getHeader();
            jam::ilda::RecordFormat rec_format = h.getFormatCode();
            
            if(rec_format == jam::ilda::RecordFormat::FORMAT_2) { // we ignore color palette frames
                continue;
            }
            h.setFormatCode(jam::ilda::RecordFormat::FORMAT_5); // 2D True Color
            frames[i].setHeader(h);
            std::vector<jam::ilda::IldaDataRecord> dr = frames[i].getDataRecords();
            for(size_t j = 0; j < dr.size(); j++) {
                uint8_t color_index = dr[j].getColorIndex();
                std::vector<number> col_vals = col->getFloatColorByIndex((size_t)color_index);
                dr[j].setRed((uint8_t)(col_vals[0] * 255.));
                dr[j].setGreen((uint8_t)(col_vals[1] * 255.));
                dr[j].setBlue((uint8_t)(col_vals[2] * 255.));
                dr[j].setPosZ(0);
            }
            frames[i].setDataRecords(dr);
            
        }
        delete col;
    }
    
    void _ildaToLaserFrames() {
        this->_laser_frames.clear();
        for(auto ilda_frame: this->_ilda_frames) {
            int frame_record_count = ilda_frame.getDataRecordCount();
                // Make new laser frame
//            auto l_frame = std::make_unique<HeliosPointHighRes[]>(frame_record_count);
            auto data_record = ilda_frame.getDataRecords();
            std::vector<HeliosPointHighRes> l_frame;
            for(int i = 0; i < frame_record_count; i++) {
                auto l_point = new HeliosPointHighRes;
                auto dr = data_record[i];
                l_point->x = dr.getPosX();
                l_point->y = dr.getPosY();
                l_point->r = dr.getRed();
                auto red = dr.getBlanking() ? 0 : this->_map((uint16_t)dr.getRed(), (uint16_t)0x0, (uint16_t)0xff, (uint16_t)0x0, (uint16_t)0xFFFF);
                auto green = dr.getBlanking() ? 0 : this->_map((uint16_t)dr.getGreen(), (uint16_t)0x0, (uint16_t)0xff, (uint16_t)0x0, (uint16_t)0xFFFF);
                auto blue = dr.getBlanking() ? 0 : this->_map((uint16_t)dr.getBlue(), (uint16_t)0x0, (uint16_t)0xff, (uint16_t)0x0, (uint16_t)0xFFFF);
                l_point->r = red;
                l_point->g = green;
                l_point->b = blue;
                l_frame.push_back(std::move(*l_point));
                
            }
            this->_laser_frames.push_back(std::move(l_frame));
        }
    }
 
    
public:
    helios(const atoms& args = {}) {
        if (!dummy()) {
            this->_ilda_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.ilda.manager"), 0, NULL);
            this->_ilda_manager_struct_ptr = (t_jam_im *)typedmess(this->_ilda_manager,symbol("get_struct"),0,0L);
            
            this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.helios.manager"), 0, NULL);
            this->_connector = (HeliosConnector *)typedmess(this->_manager,symbol("get_connector"),0,0L);
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            srand((unsigned int)ts.tv_nsec);
            this->_instance_id = rand();
            
            laser_point_t lp{0, 0};
            lpvec empty_frame{POINTS_PER_FRAME, lp};
            
            this->_setFramePoints(empty_frame);

            
            {
                std::lock_guard lock(_frame_1_lock);
                this->_frame_1 =  new HeliosPointHighRes[POINTS_PER_FRAME];
                this->_fillFrame(this->_frame_1, empty_frame);
            }
            
            {
                std::lock_guard lock(_frame_2_lock);
                this->_frame_2 =  new HeliosPointHighRes[POINTS_PER_FRAME];
                this->_fillFrame(this->_frame_2, empty_frame);
            }
            {
                std::lock_guard lock(_play_frame_lock);
                this->_play_frame = this->_frame_1;
            }
            {
                std::lock_guard lock(_edit_frame_lock);
                this->_edit_frame = this->_frame_2;
            }
           
        }
    }
    
    ~helios() {
        if (!dummy()) {
            close();
            if(this->_frame_1) {
                std::lock_guard lock(_frame_1_lock);
                delete [] this->_frame_1;
            }
            if(this->_frame_2) {
                std::lock_guard lock(_frame_2_lock);
                delete [] this->_frame_2;
            }
        }
    }
    
    MIN_DESCRIPTION { "Connect to a Helios ILDA DAC" };
    MIN_TAGS { "laser control" };
    MIN_AUTHOR{ "Jan Mech" };
    MIN_RELATED{ "jam.dmxusbpro~, jam.dmxusbpro, jam.ilda.file, jam.ilda.compose, jam.ilda.dict, jam.helios, jit.gl.sketch" };
    
    inlet<> input_1{ this, "(anything) Control Messages", "anything" };
    inlet<> input_2{ this, "(dictionary) ilda file dictionary", "dictionary" };
    outlet<> outlet_menu{ this, "(anything) Connect to umenu", "message" };
    outlet<> outlet_connected{ this, "(int) State of Connection", "int" };
    outlet<> outlet_dumpout{ this, "dumpout" };
    
    void onConnectionReset() {
        cwarn << "callback called" << endl;
    }

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
                {
                    std::lock_guard lock(_edit_frame_lock);
                    this->_fillFrame(this->_edit_frame, this->_frame_points);
                }
               
                this->_drawFrame();
            }
            return cleaned_args;
        }},
        title {"Laser Color"},
        description {"Laser Color"},
        style {c74::min::style::color},
    };
    
    attribute<fvec> scale {
        this, "scale", {1., 1.},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 2, 1.);
                
                cleaned_args[0] = std::clamp((number)cleaned_args[0], -1., 1.);
                cleaned_args[1] = std::clamp((number)cleaned_args[1], -1., 1.);
                
                this->_scaling = {cleaned_args[0], cleaned_args[1]};
                if(this->initialized()) {
                    {
                        std::lock_guard lock(_edit_frame_lock);
                        this->_fillFrame(this->_edit_frame, this->_frame_points);
                    }
                   
                    this->_drawFrame();
                }
                return cleaned_args;
            }
        },
        title {"Scale"},
        description {"Scale the output"},

    };
    
    attribute<fvec> rotate {
        this, "rotate", {0., 0., 0.},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 3, 0.);
                this->_rotation_angle = (number)cleaned_args[0] * (PI / 180.);
                cleaned_args[1] = (number)std::clamp((number)cleaned_args[1], -2., 2.);
                cleaned_args[2] = (number)std::clamp((number)cleaned_args[2], -2., 2.);
                this->_rotation_center.x = (number)cleaned_args[1];
                this->_rotation_center.y = (number)cleaned_args[2];
                if(this->initialized()) {
                    {
                        std::lock_guard lock(_edit_frame_lock);
                        this->_fillFrame(this->_edit_frame, this->_frame_points);
                    }
                    this->_drawFrame();
                }
                return cleaned_args;
            }
        },
        title {"Rotate"},
        description {"Rotate the output"},
    };
    
    attribute<symbol> drawmode {
        this, "drawmode", "direct",
        range {"direct", "ilda"},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<std::string>(args, &cleaned_args, 1, "direct");
                std::string value = cleaned_args[0];
                if(value != "direct" && value != "ilda") {
                    cleaned_args[0] = "direct";
                }
                return cleaned_args;
            }
        },
        title { "Daw Mode" },
        description { "Select between direct drawing mode and ilda file rendering." },
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
    
    message<threadsafe::no> dot {
        this, "dot", "Project a dot at position x/y",
        MIN_FUNCTION {
            if(args.size() < 2) {
                return {};
            }
            number x = std::clamp((number)args[0], -1., 1.);
            number y = std::clamp((number)args[1], -1., 1.) * -1.;
            
            this->_setFramePoints(this->_makeDotPoints(x, y));
            {
                std::lock_guard lock(_edit_frame_lock);
                this->_fillFrame(this->_edit_frame, this->_frame_points);
            }
           
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
            
            this->_setFramePoints(this->_makeEllipsePoints(center, radius));
            
            {
                std::lock_guard lock(_edit_frame_lock);
                this->_fillFrame(this->_edit_frame, this->_frame_points);
            }
           
            this->_drawFrame();
            return {};
        }
    };
    
    message<threadsafe::no> line {
        this, "line", "Project a line with from x1/y1 to x2/y2",
        MIN_FUNCTION {
            if(args.size() < 4) {
                return {};
            }
            coord_point_t start = {
                std::clamp((number)args[0], -1., 1.) * -1.,
                std::clamp((number)args[1], -1., 1.) // * -1.
            };
            
            coord_point_t end = {
                std::clamp((number)args[2], -1., 1.) * -1.,
                std::clamp((number)args[3], -1., 1.) //* -1.
            };
            
            this->_setFramePoints(this->_makeLinePoints(start, end));
            {
                std::lock_guard lock(_edit_frame_lock);
                this->_fillFrame(this->_edit_frame, this->_frame_points);
            }
           
            this->_drawFrame();
            
            
        
            return {};
        }
    };
    
    message<>ilda {
        this, "ilda", "Reference to an ILDA file.  <br/>To render a file pass in a ilda referecence to a file loaded by <o>jam.ilda.file</o> or created by <o>jam.ilda.compose</o>.",
        MIN_FUNCTION {
            if(args.size() < 1) {
                cwarn << "missing argument for message ilda" << endl;
                return {};
            }
            if(args.size() > 1) {
                cwarn << "extras argument for message ilda" << endl;
            }

            std::string ilda_file_refence = args[0];
            std::vector<jam::ilda::IldaFrame> frames = this->_getIldaManagerStructPointer()->getFrames(ilda_file_refence);
            this->_parseFrames2DToTrueColor(frames);
            this->_ilda_frames = frames;
            this->_ildaToLaserFrames();
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
                    if(this->drawmode == symbol("direct")) {
                        int status = helios->GetStatus(this->_attached_device);
                        if (status == 1) {
                            int result;
                            {
                            std::lock_guard lock(_play_frame_lock);
                            result = helios->WriteFrameHighResolution(
                                      this->_attached_device,
                                      (int)samplerate, HELIOS_FLAGS_DEFAULT,
                                      this->_play_frame, POINTS_PER_FRAME
                                     );
                            }
                            
                            if(result != HELIOS_SUCCESS) {
                                cerr << this->heliosErrorToString(result) << endl;
                            }
                            std::this_thread::sleep_for (std::chrono::milliseconds(1000 / (int)fps));
                        }
                    } else {
                        for(int i = 0; i < this->_laser_frames.size(); i++) {
                            int status = helios->GetStatus(this->_attached_device);
                            if (status == 1) {
                                unsigned int frame_size = (unsigned int)this->_laser_frames[i].size();
                                std::this_thread::sleep_for (std::chrono::milliseconds(500));
                                cout << "frame: " << i << " " << frame_size << endl;
                                int result = helios->WriteFrameHighResolution(this->_attached_device, (int)samplerate, HELIOS_FLAGS_DEFAULT, &this->_laser_frames[i][0], POINTS_PER_FRAME);
                                if(result != HELIOS_SUCCESS) {
                                    cerr << this->heliosErrorToString(result) << endl;
                                }
//                                std::this_thread::sleep_for (std::chrono::milliseconds(1000 / (int)fps));
                            }
                        }
                    }
                    
                }
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
