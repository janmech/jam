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
#include "../jam.helper/attribute_args_helper.hpp"

#define HELIOS_FILE_CHUNK 1024


using namespace c74::min;

class ildadict : public object<ildadict>
{
    
private:
    
    c74::max::t_object *_manager;                   // Pointer to global jam.ilda.manager object
                                                    // (stores data to be accasibele by other jam.ilda.* object)
    t_jam_im * _manager_struct_ptr = NULL;          // Pointer to max-object struct of the jam.ilda.manager object
    
    
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
        
        void send(ildadict* me) {
            me->_enqueue_msg_to_max(*this);
            me->deliverer_to_max.delay(0);
        }
    } queued_message_t;
    
    fifo<queued_message_t> _to_max_queue { 1000 }; // FIFO queue for messages to be sent to outlets
    std::mutex _enqueue_msg_lock;                    // Mutex lock for outlet message thread safty
    
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
    
    t_jam_im * _getStructPointer() {
        if(this->_manager_struct_ptr == NULL) {
            this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
        }
        return this->_manager_struct_ptr;
    }
    
    std::vector<jam::ilda::IldaFrame> _frames;
    
    symbol loaded_dict_name = symbol("");
    
public:
    
    ildadict(const atoms& args = {}) {
        if (args.size() > 0) {
            cwarn << "Extra argumnt for oject jam.jit.gl.dict" << endl;
        }
        this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.ilda.manager"), 0, NULL);
        this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
    };
    
    ~ildadict() {};
    
    dict d_ilda_file = dict(symbol(true));
    
    MIN_DESCRIPTION     { "Parse ILDA files to dictionary." };
    MIN_TAGS            { "ILDA, laser tools, utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file, jam.jit.gl.ilda.compose"};
    
    inlet<> input_1             { this, "ILDA file reference", "anything" };
    
    outlet<> outlet_dict      { this, "ILDA file content information as dictionary" };
    
    
    message<>bang  {
        this, "bang", "Output dictionary",
        MIN_FUNCTION {
            queued_message_t msg;
            atoms msg_atoms;
            msg_atoms.push_back("dictionary");
            msg_atoms.push_back(this->loaded_dict_name );
            
            msg.set(&outlet_dict, msg_atoms);
            msg.send(this);
            return {};
        }
    };
    
    message<>ilda {
        this, "ilda", "Reference to am ILDA file loaded by jam.ilda.file",
        MIN_FUNCTION {
            if(args.size() < 1) {
                cwarn << "missing argument for message ilda" << endl;
                return {};
            }
            std::string ilda_file_refence = args[0];
            std::vector<jam::ilda::IldaFrame> frames = this->_getStructPointer()->getFrames(ilda_file_refence);
            this->_frames = frames;
            
            this->d_ilda_file = dict(symbol(true));
            this->d_ilda_file.clear();
            this->d_ilda_file["file_name"] = this->_getStructPointer()->getFileName(ilda_file_refence);
            this->d_ilda_file["frame_count"] = (int)frames.size();
            dict d_ilda_frames(symbol(true));
            
            for (size_t frame_index = 0; frame_index < frames.size(); frame_index++) {
                dict d_ilda_frame(symbol(true));
                jam::ilda::IldaFrame f = this->_frames[frame_index];
                jam::ilda::IldaHeader h =  f.getHeader();
            
                d_ilda_frame["format_name"] = h.getFormat();
                d_ilda_frame["format_code"] = (int)h.getFormatCode();
                d_ilda_frame["company_name"] = h.getCompanyName();;
                d_ilda_frame["frame_name"] = h.getFrameName();;
                d_ilda_frame["frame_number"] = (int)h.getFrameNumber();
                d_ilda_frame["frames_in_sequence"] = (int) h.getFramesInSequence();
                d_ilda_frame["data_record_count"] = (int)h.getDataRecordCount();
                f.reset();
                
                
                std::ostringstream os;
                os << frame_index;
                d_ilda_frames[os.str()] = d_ilda_frame;
            }
            this->d_ilda_file["frames"] = d_ilda_frames;
            this->loaded_dict_name = this->d_ilda_file.name();
            queued_message_t msg;
            atoms msg_atoms;
            msg_atoms.push_back("dictionary");
            msg_atoms.push_back(this->loaded_dict_name );
            
            msg.set(&outlet_dict, msg_atoms);
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


MIN_EXTERNAL(ildadict);


