    /// @file
    ///	@ingroup    jam
    ///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
    ///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#define NANOSVG_IMPLEMENTATION

#include <iostream>
#include <string>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <queue>
#include <string>
#include "c74_min.h"
#include "jam.svg.shape.hpp"
#include "../jam.ilda_common/ilda_frame.hpp"
#include "../jam.ilda_common/ilda_header.hpp"
#include "../jam.ilda_common/ilda_data_record.hpp"
#include "../jam.ilda_common/ilda_definitions.hpp"
#include "../jam.ilda.manager/jam.ilda.manager.hpp"
#include "nanosvg.h"


using namespace c74::min;

class ildasvg : public object<ildasvg>
{
private:
    std::string _instance_id = "";                  // Unique ID for each object instance.
                                                    // Used to itentify loaded ILDA filed data in the global jam.ilda.manager
    c74::max::t_object *_manager;                   // Pointer to global jam.ilda.manager object
                                                    // (stores data to be accasibele by other jam.ilda.* object)
    t_jam_im * _manager_struct_ptr = NULL;          // Pointer to max-object struct of the jam.ilda.manager object
    
    atoms _import_args;                             // Stores arguments of import message,
                                                    // to be accasible in the scope of the file loader thread
    c74::max::t_filehandle file_handle;             // File handle for importing SVG files.
    char filename[c74::max::MAX_PATH_CHARS] = {0};  // File name of ILDA file toi be imported
    
    std::vector<jam::svg::Shape> _shapes;             // Vector of Shapes from parsed SVG file
    
    size_t _edit_frame = 0;
    
    protected :
    
        // Struct to encapsulate sending messages to outlets via the timer - for thread safty
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
        void send(ildasvg* me) {
            me->_enqueue_msg_to_max(*this);
            me->deliverer_to_max.delay(0);
        }
    } queued_message_t;
    
    fifo<queued_message_t> _to_max_queue { 1000 }; // FIFO queue for messages to be sent to outlets
    std::mutex _enqueue_msg_lock;                    // Mutex lock for outlet message thread safty
    
    bool _is_parsing = false;
    
    std::vector<jam::ilda::IldaFrame> _ilda_frames; // currently loaded/created ilda file frames
    
        // Set if the instance currently in the process of importing a file
    void _setParsingState(bool state) {
        if(state != this->_is_parsing) {
            this->_is_parsing = state;
        }
    }
    
        // get if the nstance currently in the process of importing a file
    bool _getParsingState() {
        return this->_is_parsing;
    }
    
    t_jam_im * _getStructPointer() {
        if(this->_manager_struct_ptr == NULL) {
            this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
        }
        return this->_manager_struct_ptr;
    }
    
    void _enqueue_msg_to_max(const queued_message_t &msg_to_max) {
        _enqueue_msg_lock.lock();
        this->_to_max_queue.try_enqueue(msg_to_max);
        _enqueue_msg_lock.unlock();
    }
    
        // deueueing queued outlet messages (thread safe)
    bool _dequeue_msg_to_max(queued_message_t &msg_data) {
        _enqueue_msg_lock.lock();
        bool result = this->_to_max_queue.try_dequeue(msg_data);
        _enqueue_msg_lock.unlock();
        return result;
    }
    
       // translate from normalized coordinates (-1. to 1.) to ILDA file coordinates
    int _deNormalizePosition(double pos) {
        return static_cast<int>(pos * 32000);
            //        if(pos < 0) {
            //            return static_cast<int>(pos * 32768);
            //        }
            //        return static_cast<int>(pos * 32767);
    };
    
    void _updateEditFrame() {
        if(this->_ilda_frames.size() == 0) {
            this->_edit_frame = 0;
        } else {
            if(this->_edit_frame > this->_ilda_frames.size() - 1) {
                this->_edit_frame = this->_ilda_frames.size() - 1;
            }
        }
    }
    
    void _updateOutlets() {
        this->_updateEditFrame();
        atoms msg_atoms;
        queued_message_t msg;
        
        msg_atoms.clear();
        msg_atoms.push_back(this->_ilda_frames.size());
        msg.set(&o_framecount, msg_atoms);
        msg.send(this);
        
        msg_atoms.clear();
        msg_atoms.push_back(this->_edit_frame);
        msg.set(&o_edit_frame, msg_atoms);
        msg.send(this);
        
        msg_atoms.clear();
        msg_atoms.push_back("ilda");
        msg_atoms.push_back(this->_instance_id);
        msg.set(&o_file_reference, msg_atoms);
        msg.send(this);
    }
    
    void _appendEmptyFrame() {
        jam::ilda::IldaFrame f;
        jam::ilda::IldaHeader h;
        h.setFormatCode(jam::ilda::RecordFormat::FORMAT_5);
        h.setFrameName("NOT SET");
        h.setCompanyName("NOT SET");
        h.setFrameNumber(this->_ilda_frames.size());
        h.setFramesInSequence(this->_ilda_frames.size() + 1);
        h.setIsColorPallet(false);
        h.setDataRecordCount(0);
        f.setHeader(h);
        this->_ilda_frames.push_back(f);
        this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
        this->_edit_frame = this->_ilda_frames.size() - 1;
        
        this->_updateOutlets();
    }
    
    std::vector<jam::svg::Point2D> _approximateCircle(const double c_x,const double c_y, const double r, const int segments = 100) {
        std::vector<jam::svg::Point2D> circle_points;
        for (int i = 0; i < segments; ++i) {
            jam::svg::Point2D p;
            double angle = (2.0f * PI * i) / segments;
            p.x = c_x + r * std::cos(angle);
            p.y = c_y + r * std::sin(angle);
            circle_points.push_back(p);
        }
        return circle_points;
    }
    
public:
    
    ildasvg(const atoms& args = {}) {
        if(!dummy()) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            srand((unsigned int)ts.tv_nsec);
            uint rand_id = rand();
            std::string id_string = std::to_string(rand_id);
            std::ostringstream ss;
            ss << std::setw(12) << std::setfill('0') << id_string;
            this->_instance_id = "ild_" + std::string(ss.str());
                // get the pointer to jam.ilda.manager max-object
            this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.ilda.manager"), 0, NULL);
                // get the pointer to jam.ilda.manager max-object's struct
            this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
        }
    };
    
    ~ildasvg() {};
    
    MIN_DESCRIPTION     { "Parse SVG file to ILDA file format." };
    MIN_TAGS            { "ILDA, laser tools, utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file, jam.jit.gl.ilda.frame"};
    
    inlet<> input_1             { this, "ILDA file reference", "anything" };
    outlet<> o_file_reference   { this, "ilda file reference"  };
    outlet<> o_edit_frame     { this, "Frame cureently selected for editing", "int"};
    outlet<> o_framecount       { this, "Number of frames created", "int"};
    outlet<> o_load_result      { this, "file opration success/failure notification", "list" };
    
    
    message<>bang  {
        this, "bang", "Output ILDA file reference",
        MIN_FUNCTION {
            this->_updateOutlets();
            return {};
        }
    };
    
    
    message<threadsafe::no> clear {
        this, "clear", "Remove all frames",
        MIN_FUNCTION {
            this->_ilda_frames.clear();
            this->_getStructPointer()->clearInstanceFile(this->_instance_id);
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no> seteditframe {
        this, "seteditframe", "Select frame to be edited",
        MIN_FUNCTION {
            if(args.size() == 0) {
                cwarn << "missing argument for message 'seteditframe'" << endl;
                return {};
            }
            if (args.size() > 1) {
                cwarn << "extra argument for message 'seteditframe'" << endl;
            }
            
            if(
               args[0].type() != message_type::int_argument
               && args[0].type() != message_type::float_argument) {
                   cwarn << args[0] << " bad number" << endl;
                   return {};
            }
            
            int frame_index = args[0];
            
            if(frame_index < 0 || frame_index > this->_ilda_frames.size() - 1) {
                cwarn << "frame index out of range." << endl;
                return {};
            }
            
            this->_edit_frame = frame_index;
            
            atoms msg_atoms;
            queued_message_t msg;
            
            msg_atoms.clear();
            msg_atoms.push_back(this->_edit_frame);
            msg.set(&o_edit_frame, msg_atoms);
            msg.send(this);
            
            
            return {};
        }
        
    };
    
    message<threadsafe::no> geteditframe {
        this, "geteditframe", "Outputs the currenlty selected frame for editing to the second outlet",
        MIN_FUNCTION {
            atoms msg_atoms;
            queued_message_t msg;
            
            msg_atoms.clear();
            msg_atoms.push_back(this->_edit_frame);
            msg.set(&o_edit_frame, msg_atoms);
            msg.send(this);
            return {};
        }
    };
    
    message<threadsafe::no> appendframe {
        this, "appendframe", "Append a new frame",
        MIN_FUNCTION {
            this->_appendEmptyFrame();
            return {};
        }
    };
    
    message<threadsafe::no> removeframe {
        this, "removeframe", "Remove frame by index",
        MIN_FUNCTION {
            if(args.size() == 0) {
                cwarn << "missing argument for message 'removeframe'" << endl;
                return {};
            }
            if (args.size() > 1) {
                cwarn << "extra argument for message 'removeframe'" << endl;
            }
            
            if(
               args[0].type() != message_type::int_argument
               && args[0].type() != message_type::float_argument) {
                   cwarn << args[0] << " bad number" << endl;
                   return {};
            }
            
            int frame_index = args[0];
            
            if(frame_index < 0 || frame_index > this->_ilda_frames.size() - 1) {
                cwarn << "frame index out of range." << endl;
                return {};
            }
            this->_ilda_frames.erase(this->_ilda_frames.begin() + frame_index);
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            
            if(this->_edit_frame > this->_ilda_frames.size() - 1) {
                this->_edit_frame = (this->_ilda_frames.size() > 0) ? this->_ilda_frames.size() - 1 : 0;
            }
            
            this->_updateOutlets();
            
            return {};
        }
    };
        
    message<threadsafe::no>svg {
        this, "svg", "Parse a SVG file and append as new frame",
        MIN_FUNCTION {
            if(this->_getParsingState()) {
                cwarn << "file loading already in progress" << endl;
                return {};
            }
            
            this->_setParsingState(true);
            
            if (args.size() > 1) {
                cwarn << "extra argument for message 'import'" << endl;
            }
            
            this->_import_args = args;
            atoms msg_atoms;
            queued_message_t msg;
            
            
            short path;
            short open_result;
            c74::max::t_fourcc filetype = 'SVG', outtype;
            
            if (this->_import_args.size() == 0) {
                open_result = c74::max::open_dialog(filename, &path, &outtype, &filetype, (short)1);
                if(open_result != c74::max::MAX_ERR_NONE) {
                    cerr << "couldn't open file" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("svg");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_load_result, msg_atoms);
                    msg.send(this);
                    this->_setParsingState(false);
                    return{};
                }
            }
            else {
                std::string user_filename = this->_import_args[0];
                if(user_filename.size() > c74::max::MAX_PATH_CHARS - 1) {
                    cerr << "file name too long" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("svg");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_load_result, msg_atoms);
                    msg.send(this);
                    this->_setParsingState(false);
                    return {};
                    
                }
                strcpy(filename, user_filename.c_str());
                
                open_result = c74::max::locatefile_extended(filename, &path, &outtype, &filetype, (short)1);
                if(open_result != c74::max::MAX_ERR_NONE) {
                    cerr << "couldn't open file" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("svg");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_load_result, msg_atoms);
                    msg.send(this);
                    this->_setParsingState(false);
                    return {};
                }
            }
            
            open_result = c74::max::path_opensysfile( filename, path, &file_handle,c74::max::READ_PERM);
            
            if(open_result != c74::max::MAX_ERR_NONE) {
                cerr << "couldn't open file" << endl;
                msg_atoms.clear();
                msg_atoms.push_back("svg");
                msg_atoms.push_back(filename);
                msg_atoms.push_back(0);
                msg.set(&o_load_result, msg_atoms);
                msg.send(this);
                this->_setParsingState(false);
                return {};
            }
            
            
            unsigned long size;
            c74::max::t_max_err read_result = 0;
            c74::max::t_handle file_content_handle = nullptr;
            std::string file_content = "";
            c74::max::sysfile_geteof(file_handle,&size);
            
            size = c74::max::sysmem_handlesize(file_content_handle);
            
            if (!(file_content_handle = c74::max::sysmem_newhandle(size))) {
                cerr << "not enough memory to open " << filename << endl;
                msg_atoms.clear();
                msg_atoms.push_back("svg");
                msg_atoms.push_back(filename);
                msg_atoms.push_back(0);
                msg.set(&o_load_result, msg_atoms);
                msg.send(this);
                this->_setParsingState(false);
                return {};
            }
            
                // https://cycling74.com/forums/t_handle-and-sysmem_newhandle-crash-help-needed
            read_result = c74::max::sysfile_readtextfile(file_handle,file_content_handle,size, c74::max::TEXT_ENCODING_USE_FILE);
            if(read_result != c74::max::MAX_ERR_NONE) {
                c74::max::sysmem_freehandle(file_content_handle);
                cerr << "couldn't read file" << endl;
                msg_atoms.clear();
                msg_atoms.push_back("svg");
                msg_atoms.push_back(filename);
                msg_atoms.push_back(0);
                msg.set(&o_load_result, msg_atoms);
                msg.send(this);
                this->_setParsingState(false);
                return {};
            }
            
            file_content = *file_content_handle;
            NSVGimage* image;
                // Load SVG
            image = nsvgParse(*file_content_handle, "px", 96);
            if(image == NULL) {
                c74::max::sysmem_freehandle(file_content_handle);
                cerr << "couldn't parse file" << endl;
                msg_atoms.clear();
                msg_atoms.push_back("svg");
                msg_atoms.push_back(filename);
                msg_atoms.push_back(0);
                msg.set(&o_load_result, msg_atoms);
                msg.send(this);
                this->_setParsingState(false);
                return {};
                
            }
        
            msg_atoms.clear();
            msg_atoms.push_back("svg");
            msg_atoms.push_back(filename);
            msg_atoms.push_back(1);
            msg.set(&o_load_result, msg_atoms);
            msg.send(this);
            this->_setParsingState(false);
            
            
            // parse SVG data into shapes
            _shapes.clear();
            for (NSVGshape* shape = image->shapes; shape != nullptr; shape = shape->next) {
                jam::svg::Shape s("SVGPath");
                
                if(shape->stroke.type == NSVG_PAINT_COLOR) {
                        // AAAAAAAA BBBBBBB  GGGGGGGG RRRRRRRR
                        // 11111111 11111111 11111111 11111111
                    unsigned int strokeColor = shape->stroke.color;
                    strokeColor = strokeColor & 0x00FFFFFF; // Eliminate Alpha
                    unsigned int r = (strokeColor &  0x000000FF);
                    unsigned int g = (strokeColor &  0x0000FF00) >> 8;
                    unsigned int b = (strokeColor &  0x00FF0000) >> 16;
                        // Black doesn't exist in lasers, so we change it to white
                    if(r + g + g == 0) {
                        r = g = b = 255;
                    }
                    jam::svg::RGBColor color;
                    color.r = static_cast<uint8_t>(r);
                    color.g = static_cast<uint8_t>(g);
                    color.b = static_cast<uint8_t>(b);
                    s.setColor(color);
                }
                
                for (NSVGpath* path = shape->paths; path != nullptr; path = path->next) {
                    for (int i = 0; i < path->npts; ++i) {
                        float x = path->pts[i * 2];       // x coordinate
                        float y = path->pts[i * 2 + 1];   // y coordinate
                                                          // Y-Axis Flip
                        y = image->height - y;
                            // Normalize to [-1, 1]
                        float nx = (x / image->width) * 2.0f - 1.0f;
                        float ny = (y / image->height) * 2.0f - 1.0f;
                        s.addPoint({nx, ny});
                    }
                    
                    if (!s.getPoints().empty()) {
                            // Close path if marked closed
                        if (path->closed) {
                            s.addPoint(s.getPoints().front());
                        }
                        s.thinShape();
                    }
                    this->_shapes.push_back(s);
                }
                
            }
            
            nsvgDelete(image);
            
            // Add shape data to current edit_frame
            if(this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
                
            jam::ilda::IldaFrame f = this->_ilda_frames[this->_edit_frame];
            for(size_t i = 0; i< this->_shapes.size(); i++) {
                for(size_t j = 0; j < this->_shapes[i].getPoints().size(); j++) {
                    bool is_blanking = (j == 0);
                    jam::svg::Point2D p = this->_shapes[i].getPoints()[j];
                    jam::ilda::IldaDataRecord dr;
                    dr.setRed(is_blanking ? 0 : this->_shapes[i].getColor().r);
                    dr.setGreen(is_blanking ? 0 : this->_shapes[i].getColor().g);
                    dr.setBlue(is_blanking ? 0 : this->_shapes[i].getColor().b);
                    dr.setBlanking(is_blanking);
                    dr.setPosX(this->_deNormalizePosition(p.x));
                    dr.setPosY(this->_deNormalizePosition(p.y));
                    f.pushRecord(dr);
                }
                
            }
            this->_ilda_frames[this->_edit_frame] = f;
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no>line {
        this, "line", "Draw a line into a frame",
        MIN_FUNCTION {
            if(args.size() < 4) {
                cwarn << "missing argument for message 'line'" << endl;
                return {};
            }
            if (this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            double x_start = args[0];
            double y_start = args[1];
            double x_end   = args[2];
            double y_end   = args[3];
            
            uint8_t r = 255;
            uint8_t g = 255;
            uint8_t b = 255;
            if(args.size() >= 7) {
                r = uint8_t((float)args[4] * 255.);
                g = uint8_t((float)args[5] * 255.);
                b = uint8_t((float)args[6] * 255.);
            }
            // move to staring point
            jam::ilda::IldaDataRecord r_start;
            r_start.setRed(0);
            r_start.setGreen(0);
            r_start.setBlue(0);
            r_start.setPosX(this->_deNormalizePosition(x_start));
            r_start.setPosY(this->_deNormalizePosition(y_start));
            r_start.setBlanking(true);
            
            jam::ilda::IldaDataRecord r_end;
            r_end.setRed(r);
            r_end.setGreen(g);
            r_end.setBlue(b);
            r_end.setPosX(this->_deNormalizePosition(x_end));
            r_end.setPosY(this->_deNormalizePosition(y_end));
            r_end.setBlanking(false);
            
            this->_ilda_frames[this->_edit_frame].pushRecord(r_start);
            this->_ilda_frames[this->_edit_frame].pushRecord(r_end);
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            
            this->_updateOutlets();
            
            
            return {};
        }
    };
    
    message<threadsafe::no>circle {
        this, "circle", "Draw a circle into a frame",
        MIN_FUNCTION {
            if(args.size() < 3) {
                cwarn << "missing argument for message 'circle'" << endl;
                return {};
            }
            if (this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            
            double x_center = args[0];
            double y_center = args[1];
            double radius   = args[2];
            
            uint8_t r = 255;
            uint8_t g = 255;
            uint8_t b = 255;
            if(args.size() >= 6) {
                r = uint8_t((float)args[3] * 255.);
                g = uint8_t((float)args[4] * 255.);
                b = uint8_t((float)args[5] * 255.);
            }
            
            int segments = 100;
            if(args.size() >= 7) {
                segments = (int)args[6];
                segments = (segments < 3) ? 3 : segments;
                segments = (segments > 200) ? 200 : segments;
            }
            std::vector<jam::svg::Point2D> circle_points = this->_approximateCircle(x_center, y_center, radius, segments);
            for(size_t i = 0; i < circle_points.size(); i++) {
                jam::ilda::IldaDataRecord dr;
                uint8_t dr_r = (i == 0) ? 0 : r;
                uint8_t dr_g = (i == 0) ? 0 : g;
                uint8_t dr_b = (i == 0) ? 0 : b;
                bool is_blanking = (i == 0);
                dr.setRed(dr_r);
                dr.setGreen(dr_g);
                dr.setBlue(dr_b);
                dr.setPosX(this->_deNormalizePosition(circle_points[i].x));
                dr.setPosY(this->_deNormalizePosition(circle_points[i].y));
                dr.setBlanking(is_blanking);
                this->_ilda_frames[this->_edit_frame].pushRecord(dr);
            }
            
            // close shape
            jam::ilda::IldaDataRecord dr;
            dr.setRed(r);
            dr.setGreen(g);
            dr.setBlue(b);
            dr.setPosX(this->_deNormalizePosition(circle_points[0].x));
            dr.setPosY(this->_deNormalizePosition(circle_points[0].y));
            dr.setBlanking(false);
            this->_ilda_frames[this->_edit_frame].pushRecord(dr);
            
            
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
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


MIN_EXTERNAL(ildasvg);


