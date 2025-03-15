    /// @file
    ///	@ingroup    jam
    ///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
    ///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#include <iostream>
#include <string>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <queue>
#include <thread>
#include <string>
#include "c74_min.h"
#include  "../jam.ilda_common/ilda_definitions.hpp"
#include "../jam.ilda_common/ilda_colors.hpp"
#include "../jam.ilda.manager/jam.ilda.manager.hpp"
#include "../jam.ilda_common/ilda_frame.hpp"
#include "../jam.ilda_common/ilda_header.hpp"
#include "../jam.ilda_common/ilda_data_record.hpp"
#include "../jam.ilda_common/ilda_colors.hpp"

#define HELIOS_FILE_CHUNK 1024


using namespace c74::min;

class ildaframe : public object<ildaframe>
{
    
private:
    
    c74::max::t_object *_sketch_object;             // Pointer to a the jit.gl.sketch object instance,
                                                    // which actually handles the rendering of the frames
    c74::max::t_object *_manager;                   // Pointer to global jam.ilda.manager object
                                                    // (stores data to be accasibele by other jam.ilda.* object)
    t_jam_im * _manager_struct_ptr = NULL;          // Pointer to max-object struct of the jam.ilda.manager object
    
    std::vector<jam::ilda::IldaFrame> _frames;

    
    
protected:
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
        void send(ildaframe* me) {
            me->_enqueue_msg_to_max(*this);
            me->deliverer_to_max.delay(0);
        }
    } queued_message_t;
    
    fifo<queued_message_t> _to_max_queue_2 { 1000 }; // FIFO queue for messages to be sent to outlets
    std::mutex _enqueue_msg_lock;                    // Mutex lock for outlet message thread safty
    
    void _enqueue_msg_to_max(const queued_message_t &msg_to_max) {
        _enqueue_msg_lock.lock();
        this->_to_max_queue_2.try_enqueue(msg_to_max);
        _enqueue_msg_lock.unlock();
    }
    
        // deueueing queued outlet messages (thread safe)
    bool _dequeue_msg_to_max(queued_message_t &msg_data) {
        _enqueue_msg_lock.lock();
        bool result = this->_to_max_queue_2.try_dequeue(msg_data);
        _enqueue_msg_lock.unlock();
        return result;
    }
    
    float _normalizePosition(int pos) {
        pos = (pos < -32768) ? -32768 : pos;
        pos = (pos > 32767) ? 32767 : pos;
        if (pos > 0) {
            return ((float)pos / 32767.) * 0.85;
        }
        return ((float)pos / 32768) * 0.85;
    };
    
    t_jam_im * _getStructPointer() {
        if(this->_manager_struct_ptr == NULL) {
            this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
        }
        return this->_manager_struct_ptr;
    }
    
public:
    
    ildaframe(const atoms& args = {}) {
        if (args.size() > 1) {
            cout << "TEST" << endl;
            cwarn << "Extra argumnt for oject jam.jit.gl.frame" << endl;
        }
        
            // TODO: add attribute draw_to and set vale from argument
        
        
        atom argu;
        this->_sketch_object = (c74::max::t_object*)c74::max::newinstance(symbol("jit.gl.sketch"), 0, &argu);
        this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.ilda.manager"), 0, NULL);
        this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
    }
    
    ~ildaframe() {
        c74::max::freeobject(this->_sketch_object);
    }
    
    static constexpr const char* my_description {"ildaframe"};
    
    MIN_DESCRIPTION     { "ildaframe" };
    
    MIN_TAGS            { "utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file"};
    
    inlet<> input_1             { this, "(anything) Control Messages", "anything" };
    outlet<> output_loaded      { this, "bang when done loading file", "bang" };
    outlet<> output_dumpout     { this, "dumpout" };
    
    
    message<>bang  {
        this, "bang", "trigger output",
        MIN_FUNCTION {
            if(this->_frames.size() == 0) {
                cwarn << "no data" << endl;
                return {};
            }
            
            return {};
        }
    };
    
    message<>drawto {
        this, "drawto", "jitter context",
        MIN_FUNCTION {
            atom mess_arg =args[0];
                typedmess(this->_sketch_object,symbol("drawto"),1,&mess_arg);
            return {};
        }
    };
    
    message<>reset {
        this, "reset", "reset",
        MIN_FUNCTION {
                typedmess(this->_sketch_object,symbol("reset"),0,0L);
            return {};
        }
    };
    
    message<>framecount {
        this, "framecount", "Number of frames in the currently loaded ILDA file.",
        MIN_FUNCTION {
            queued_message_t msg;
            atoms msg_atoms;
            msg_atoms.push_back("framecount");
            msg_atoms.push_back(this->_frames.size());
            msg.set(&output_dumpout,msg_atoms);
            msg.send(this);
            
            return {};
            
        }
    };
    
    message<> frame {
        this, "frame", "render frame",
        MIN_FUNCTION {
            if(this->_frames.size() == 0) {
                cwarn << "no data" << endl;
                return {};
            }
            
            if(args.size() < 1) {
                cwarn << "missing argument for message frame" << endl;
                return {};
            }
            if (args.size() < 1) {
                cwarn << "extra argument for message frame" << endl;
            }
            int frame_index = (int)args[0];
            if(frame_index < 0) {
                return {};
            }
            if (frame_index > this->_frames.size() -1 ) {
                return {};
            }
            jam::ilda::IldaFrame frame = this->_frames.at(frame_index);
            
            int format_code = frame.getHeader().getFormatCode();
            
            bool is_indexed_color = (
                                     format_code == (int)jam::ilda::RecordFormat::FORMAT_0
                                     || format_code ==  (int)jam::ilda::RecordFormat::FORMAT_1
                                     );
            typedmess(this->_sketch_object,symbol("reset"),0,0L);
            
            atom args_color_values[3] = {atom(1.),atom(1.),atom(1.)};
            atom sketch_args[5] = {atom(1), atom(1.), atom(0.), atom(0.), atom(0.) };
            
            typedmess(this->_sketch_object,symbol("glcolor"),4,sketch_args);
            
            sketch_args[0] = atom("glcolor");
            sketch_args[1] = atom(1);
            typedmess(this->_sketch_object,symbol("cmd_enable"),2,sketch_args);
            
            sketch_args[0] = atom(2.);
            typedmess(this->_sketch_object,symbol("gllinewidth"),1,sketch_args);
            
            frame.reset();
            jam::ilda::IldaDataRecord data_record;
            while(frame.getNext(&data_record)) {
                
                if(is_indexed_color) {
                    uint8_t color_index = data_record.getColorIndex();
                    
                    std::vector<float> color_values = jam::ilda::Colors::getFloatColorByIndex(color_index);
                    
                    args_color_values[0] = atom(color_values[0]);
                    args_color_values[1] = atom(color_values[1]);
                    args_color_values[2] = atom(color_values[2]);
                } else {
                    uint8_t red = data_record.getRed();
                    uint8_t green = data_record.getGreen();
                    uint8_t blue = data_record.getBlue();
                    args_color_values[0] = atom((float) red / 255.);
                    args_color_values[1] = atom((float) green / 255.);
                    args_color_values[2] = atom((float) blue / 255.);
                }
                
                bool blanking = data_record.getBlanking();

                int pos_x = data_record.getPosX();
                int pos_y = data_record.getPosY();
                sketch_args[0] = atom(this->_normalizePosition(pos_x));
                sketch_args[1] = atom(this->_normalizePosition(pos_y));
                sketch_args[2] = atom(0); // For now we are ignoring z axis
                
                if(blanking) {
                    typedmess(this->_sketch_object,symbol("moveto"),3,sketch_args);
                    
                } else {
                    typedmess(this->_sketch_object,symbol("glcolor"),3,args_color_values);
                    typedmess(this->_sketch_object,symbol("lineto"),3,sketch_args);
                }

            }
            return {};
        }
    };
    
    message<>ilda {
        this, "ilda", "reference to am ILDA file loaded by jam.ilda.file",
        MIN_FUNCTION {
            if(args.size() < 1) {
                cwarn << "missing argument for message ilda" << endl;
                return {};
            }
            if(args.size() > 1) {
                cwarn << "extras argument for message ilda" << endl;
            }
            std::string ilda_file_refence = args[0];
            std::vector<jam::ilda::IldaFrame> frames = this->_getStructPointer()->getFrames(ilda_file_refence);
            this->_frames = frames;
            
            queued_message_t msg;
            atoms msg_atoms;
            msg_atoms.push_back("bang");
            msg.set(&output_loaded,msg_atoms);
            msg.send(this);
           
            return {};
               
        }
    };
    
    // Timer dedicated to deliver messages to object outlets
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


MIN_EXTERNAL(ildaframe);
