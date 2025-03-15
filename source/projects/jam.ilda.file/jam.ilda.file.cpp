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
#include <string>
#include <functional>
#include <map>
#include <chrono>
#include "c74_min.h"
#include "../jam.ilda_common/ilda_file_processor.hpp"
#include "../jam.ilda.manager/jam.ilda.manager.hpp"


#define HELIOS_FILE_CHUNK 1024

using namespace c74::min;

class ildafile : public object<ildafile>
{
    
private:
    std::string _instance_id = "";                  // Unique ID for each object instance.
                                                    // Used to itentify loaded ILDA filed data in the global jam.ilda.manager
    atoms _import_args;                             // Stores arguments of import message,
                                                    // to be accasible in the scope of the file loader thread
    c74::max::t_filehandle ilda_file_handle;        // File handle for importing ILDA files.
    char filename[c74::max::MAX_PATH_CHARS] = {0};  // File name of ILDA file toi be imported
    c74::max::t_object *_manager;                   // Pointer to global jam.ilda.manager object
                                                    // (stores data to be accasibele by other jam.ilda.* object)
    t_jam_im * _manager_struct_ptr = NULL;     // Pointer to max-object struct of the jam.ilda.manager object
    
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
        void send(ildafile* me) {
            me->_enqueue_msg_to_max(*this);
            me->deliverer_to_max.delay(0);
        }
    } queued_message_t;
    
    jam::ilda::IldaFileProcessor _fileProcessor;     // Class with functions for ILDA file processing/parsing
    fifo<queued_message_t> _to_max_queue_2 { 1000 }; // FIFO queue for messages to be sent to outlets
    std::mutex _enqueue_msg_lock;                    // Mutex lock for outlet message thread safty
    std::thread _device_scan_thread;                 // Thread for parsing ILDA file asynchronously
    
    bool _is_parsing = false;
    
    t_jam_im * _getStructPointer() {
        if(this->_manager_struct_ptr == NULL) {
            this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
        }
        return this->_manager_struct_ptr;
    }
    
        // Set if the instance currently in the process of importing a file
    void _setParingState(bool state) {
        if(state != this->_is_parsing) {
            this->_is_parsing = state;
        }
    }
        // get if the nstance currently in the process of importing a file
    bool _getParsingState() {
        return this->_is_parsing;
    }
    
        // enqueueing queued outlet messages (thread safe)
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
    
    
    
public:
    
    
    ildafile(const atoms& args = {}) {
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
    }
    
    ~ildafile() {}
    
    static constexpr const char* my_description {"foo"};
    
    MIN_DESCRIPTION     { "Connect to the Helios ILDA DAC" };
    
    MIN_TAGS            { "utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.helios"};
    
    inlet<> input_1    { this, "(anything) Control Messages", "anything" };
    outlet<> output_1   { this, "fileinfo"  };
    outlet<> output_3   { this, "file opration success/failure notification", "list" };
    
    message<> bang {
        this, "bang", "Output the the reference to loaded ILDA file",
        MIN_FUNCTION {
            if(!this->_fileProcessor.fileLoaded()) {
                cwarn << "No file loaded." << endl;
            }
            output_1("ilda", this->_instance_id);
            return {};
        }
    };
    
    message<threadsafe::no>import {
        this, "import", "import an ILDA file",
        MIN_FUNCTION {
            if(this->_getParsingState()) {
                cwarn << "file loading already in progress" << endl;
                return {};
            }
            
            this->_setParingState(true);
            
            if (args.size() > 1) {
                cwarn << "extra argument for message 'import'" << endl;
            }
            
            this->_import_args = args;
            atoms msg_atoms;
            queued_message_t msg;
            
            
            short path;
            short open_result;
            c74::max::t_fourcc filetype = 'ILDA', outtype;
            
            if (this->_import_args.size() == 0) {
                open_result = c74::max::open_dialog(filename, &path, &outtype, &filetype, (short)1);
                if(open_result != 0) {
                    cerr << "Couldn't open file" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("import");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&output_3, msg_atoms);
                    msg.send(this);
                    this->_setParingState(false);
                    return{};
                }
            }
            else {
                std::string user_filename = this->_import_args[0];
                if(user_filename.size() > c74::max::MAX_PATH_CHARS - 1) {
                    cerr << "file name too long" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("import");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&output_3, msg_atoms);
                    msg.send(this);
                    this->_setParingState(false);
                    return {};
                    
                }
                strcpy(filename, user_filename.c_str());
                
                open_result = c74::max::locatefile_extended(filename, &path, &outtype, &filetype, (short)1);
                if(open_result != 0) {
                    cerr << "Couldn't open file" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("import");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&output_3, msg_atoms);
                    msg.send(this);
                    this->_setParingState(false);
                    return {};
                }
            }
            
            open_result = c74::max::path_opensysfile( filename, path, &ilda_file_handle,c74::max::READ_PERM);
            
            if(open_result != 0) {
                cerr << "Couldn't open file" << endl;
                msg_atoms.clear();
                msg_atoms.push_back("import");
                msg_atoms.push_back(filename);
                msg_atoms.push_back(0);
                msg.set(&output_3, msg_atoms);
                msg.send(this);
                this->_setParingState(false);
                return {};
            }
            
            this->_device_scan_thread = std::thread([this]() {
                auto b = this->box();
                number current_progress {-1.};
                b("startprogress", &current_progress);
                atoms msg_atoms;
                queued_message_t msg;
                this->_setParingState(true);
                
                
                std::vector<char> ilda_file_bytes;
                char file_buffer[HELIOS_FILE_CHUNK];
                c74::max::t_ptr_size chunk_size = HELIOS_FILE_CHUNK;
                c74::max::t_max_err read_result = 0;
                while(true) {
                    read_result = c74::max::sysfile_read(ilda_file_handle,&chunk_size,file_buffer);
                    for(size_t i = 0; i < chunk_size; i++) {
                        ilda_file_bytes.push_back(file_buffer[i]);
                    }
                    if (read_result < 0) {
                        break;
                    }
                    
                }
                    // Set, parse and validate file date
                this->_fileProcessor.setFileData(ilda_file_bytes);
                jam::ilda::ParseResult result = this->_fileProcessor.parseFileData();
                int success = 1;
                if(result != jam::ilda::ParseResult::SUCCESS) {
                    success = 0;
                    this->_fileProcessor.clearFileData();
                } else {
                    std::vector<jam::ilda::IldaFrame> frames =  this->_fileProcessor.getFrames();
                    this->_getStructPointer()->setInstanceFile(this->_instance_id, frames);
                    
                    std::vector<jam::ilda::IldaFrame> foo = this->_getStructPointer()->getFrames(this->_instance_id);
                    msg_atoms.clear();
                    msg_atoms.push_back("ilda");
                    msg_atoms.push_back(this->_instance_id);
                    msg.set(&output_1, msg_atoms);
                    msg.send(this);
                }
                
                msg_atoms.clear();
                msg_atoms.push_back("import");
                msg_atoms.push_back(filename);
                msg_atoms.push_back(success);
                msg.set(&output_3, msg_atoms);
                msg.send(this);
                
                b("stopprogress");
                
                this->_setParingState(false);
            });
            
            
            this->_device_scan_thread.detach();
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


MIN_EXTERNAL(ildafile);
