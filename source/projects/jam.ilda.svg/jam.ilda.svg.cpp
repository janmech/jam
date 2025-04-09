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
    
    std::vector<jam::svg::Shape> shapes;                      // Vector of Shapes from parsed SVG file
    
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
    outlet<> o_load_result   { this, "file opration success/failure notification", "list" };
    
    
    message<>bang  {
        this, "bang", "test",
        MIN_FUNCTION {
            cout << "Bandg Test" << endl;
            return {};
        }
    };
    
    message<threadsafe::no>open {
        this, "open", "open a SVG file",
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
                    msg_atoms.push_back("open");
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
                    msg_atoms.push_back("open");
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
                    msg_atoms.push_back("open");
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
                msg_atoms.push_back("open");
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
                msg_atoms.push_back("open");
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
                msg_atoms.push_back("open");
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
                msg_atoms.push_back("open");
                msg_atoms.push_back(filename);
                msg_atoms.push_back(0);
                msg.set(&o_load_result, msg_atoms);
                msg.send(this);
                this->_setParsingState(false);
                return {};
                
            }
            cout << "size: " << image->width << " x " << image->height << endl;
            
            msg_atoms.clear();
            msg_atoms.push_back("open");
            msg_atoms.push_back(filename);
            msg_atoms.push_back(1);
            msg.set(&o_load_result, msg_atoms);
            msg.send(this);
            this->_setParsingState(false);
            
            
            shapes.clear();
            for (NSVGshape* shape = image->shapes; shape != nullptr; shape = shape->next) {
                jam::svg::Shape s("SVGPath");
                

                if(shape->stroke.type == NSVG_PAINT_COLOR) {
                    // AAAAAAAA BBBBBBB  GGGGGGGG RRRRRRRR
                    // 11111111 11111111 11111111 11111111
                    volatile unsigned int strokeColor = shape->stroke.color;
                    strokeColor = strokeColor & 0x00FFFFFF; // Eliminate Alpha
                    volatile unsigned int r = (strokeColor &  0x000000FF);
                    volatile unsigned int g = (strokeColor &  0x0000FF00) >> 8;
                    volatile unsigned int b = (strokeColor &  0x00FF0000) >> 16;
                    // Black doesn't exist in lasers, so we change it to white
                    if(r + g + g == 0) {
                        r = g = b = 255;
                    }
                    s.color.r = static_cast<double>(r) / 255.;
                    s.color.g = static_cast<double>(g) / 255.;
                    s.color.b = static_cast<double>(b) / 255.;
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
                    
                    if (!s.points.empty()) {
                            // Close path if marked closed
                        if (path->closed) {
                            s.addPoint(s.points.front());
                        }
                        s.thinShape();
                    }
                    shapes.push_back(s);
                }
            }
            nsvgDelete(image);
            
            
            
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


