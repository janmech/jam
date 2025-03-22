    /// Definition of the jam.ilda.manager data structures.
    /// Unlike the convention this is not done in the c file, because other object files need acces to this as well
#include "c74_min_api.h"
#include "../jam.ilda_common/ilda_definitions.hpp"
#include "../jam.ilda_common/ilda_frame.hpp"
#include <map>
#include <string>
#include <mutex>



#ifndef ilda_manager_h
#define ilda_manager_h

typedef struct _instance_file {
    std::string _file_name = "";
    std::string _owner_id = "";
    std::vector<jam::ilda::IldaFrame> _frames;
} t_instance_file;

typedef struct _jam_im
{
    std::mutex _file_access_lock;
    c74::max::t_object s_obj;     // t_object header
    std::vector<t_instance_file> _files;
    
    void setInstanceFile(
                         std::string owner_id,
                         std::vector<jam::ilda::IldaFrame> frames,
                         std::string file_name = ""
                         ) {
        
        t_instance_file f;
        f._owner_id = owner_id;
        f._frames = frames;
        f._file_name = file_name;
        int owner_index = -1;
        
        
        for(int i = 0; i < _files.size(); i++) {
            if(_files[i]._owner_id == owner_id) {
                owner_index = i;
                break;
            }
        }
        if(owner_index == -1) {
            _files.push_back(f);
        } else {
            _files[owner_index] = f;
        }
    }
    
    void clearInstanceFile(std::string owner_id) {
        for(int i = 0; i < _files.size(); i++) {
            if(_files[i]._owner_id == owner_id) {
                _files[i]._frames.clear();
                break;
            }
        }
    }
    
    std::vector<jam::ilda::IldaFrame> getFrames(std::string owner_id) {
        std::vector<jam::ilda::IldaFrame> f;
        for(size_t i = 0; i < _files.size(); i++) {
            if(_files[i]._owner_id == owner_id) {
                f = _files[i]._frames;
                break;
            }
        }
        return f;
    }
    
    std::string getFileName(std::string owner_id) {
        std::string f = "";
        for(size_t i = 0; i < _files.size(); i++) {
            if(_files[i]._owner_id == owner_id) {
                f = _files[i]._file_name;
                break;
            }
        }
        return f;
    }
    
} t_jam_im;
#endif


