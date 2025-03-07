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

#define OBJECT_VERSION "jam.helios v.0.0.0"
#define HELIOS_FILE_CHUNK 1024


using namespace c74::min;

class ildasketch : public object<ildasketch>
{
    
    
public:
    
    ildasketch(const atoms& args = {}) {}
    
    ~ildasketch() {
        
    }
    
    static constexpr const char* my_description {"ildasketch"};
    
    MIN_DESCRIPTION     { "ildasketch" };
    
    MIN_TAGS            { "utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file"};
    
    inlet<> input_1    { this, "(anything) Control Messages", "anything" };
    outlet<> output_1   { this, "(list) jit.gl.sketch draw commands", "list"  };
    
    message<>bang  {
        this, "bang", "trigger output",
        MIN_FUNCTION {
            cout << "bang ilda frame dictionary" << endl;
            
            return {};
        }
    };
    
    message<>dictionary {
        this, "dictionary", "ilda frame dictionary",
        MIN_FUNCTION {
            cout << "ilda frame dictionary" << endl;
            dict frame_dict = {args[0]};
            int data_record_count = 0;
            try {
                std::string count_string = atom(frame_dict.at("data_record_count"));
                
                data_record_count = std::atoi(count_string.c_str());
        
                cout << "data_records: " << data_record_count << endl;
            } catch(...) {
                cerr << "error parsing frame dict." << endl;
                return {};
            }
            
            c74::min::symbol key {"data_records"};
            auto data_records_dict_atom = c74::min::atom(frame_dict[key].begin());
            
                // Create an unregistered subdict from the atom
            dict data_records_dict {data_records_dict_atom};
        
            try {
                output_1("glcolor", 1., 1., 0., 1);
                output_1("cmd_enable", "glcolor", 1);
                
                for(int i = 0; i < (int)data_record_count; i++) {
                    auto data_record_dict = data_records_dict.at(symbol(i));
                    // TODO: Continue here
                }
            } catch (std::runtime_error& e) {
                cerr << "error parsing frame dict." << endl;
            }
            return {};
        }
    };
    
private:
    
    std::vector<float> _getFloatColorByIndex(uint8_t color_index) {
        std::vector<float> color_vector;
        int offset = (int)color_index * 3;
        color_vector.push_back((float)this->_ilda_color_pallet[offset] / 255.);
        color_vector.push_back((float)this->_ilda_color_pallet[offset+1] / 255.);
        color_vector.push_back((float)this->_ilda_color_pallet[offset+2] / 255.);
        return color_vector;
    };
    
    float _scalePosition(int pos) {
        pos = (pos < -32768) ? -32768 : pos;
        pos = (pos > 32767) ? 32767 : pos;
        if (pos > 0) {
            return (float)pos / 32767.;
        }
        return (float)pos / 32768;
    };
    
    uint8_t _ilda_color_pallet[768] = {
        0,0,0,255,255,255,255,0,0,255,255,0,0,255,0,0,255,255,0,0,255,255,0,255,255,128,128,255,140,128,255,151,128,255,163,128,255,174,128,255,186,128,255,197,128,255,209,128,255,220,128,255,232,128,255,243,128,255,255,128,243,255,128,232,255,128,220,255,128,209,255,128,197,255,128,186,255,128,174,255,128,163,255,128,151,255,128,140,255,128,128,255,128,128,255,140,128,255,151,128,255,163,128,255,174,128,255,186,128,255,197,128,255,209,128,255,220,128,255,232,128,255,243,128,255,255,128,243,255,128,232,255,128,220,255,128,209,255,128,197,255,128,186,255,128,174,255,128,163,255,128,151,255,128,140,255,128,128,255,140,128,255,151,128,255,163,128,255,174,128,255,186,128,255,197,128,255,209,128,255,220,128,255,232,128,255,243,128,255,255,128,255,255,128,243,255,128,232,255,128,220,255,128,209,255,128,197,255,128,186,255,128,174,255,128,163,255,128,151,255,128,140,255,0,0255,23,0,255,46,0,255,70,0,255,93,0,255,116,0,255,139,0,255,162,0,255,185,0,255,209,0,255,232,0,255,255,0,232,255,0,209,255,0,185,255,0,162,255,0,139,255,0,116,255,0,93,255,0,70,255,0,46,255,0,23,255,0,0,255,00,255,23,0,255,46,0,255,70,0,255,93,0,255,116,0,255,139,0,255,162,0,255,185,0,255,209,0,255,232,0,255,255,0,232,255,0,209,255,0,185,255,0,162,255,0,139,255,0,116,255,0,93,255,0,70,255,0,46,255,0,23,255,0,0,255,23,0,255,46,0,255,70,0,255,93,0,255,116,0,255,139,0,255,162,0,255,185,0,255,209,0,255,232,0,255,255,0,255,255,0,232,255,0,209,255,0,185,255,0,162,255,0,139,255,0,116,255,0,93,255,0,70,255,0,46,255,0,23,128,0,0,128,12,0,128,23,0,128,35,0,128,47,0,128,58,0,128,70,0,128,81,0,128,93,0,128,105,0,128,116,0,128,128,0,116,128,0,105,128,0,93,128,0,81,128,0,70,128,0,58,128,0,47,128,0,35,128,0,23,128,0,12,128,0,0,128,0,0,128,12,0,128,23,0,128,35,0,128,47,0,128,58,0,128,70,0,128,81,0,128,93,0,128,105,0,128,116,0,128,128,0,116,128,0,105,128,0,93,128,0,81,128,0,70,128,0,58,128,0,47,128,0,35,128,0,23,128,0,12,128,0,0,128,12,0,128,23,0,128,35,0,128,47,0,128,58,0,128,70,0,128,81,0,128,93,0,128,105,0,128,116,0,128,128,0,128,128,0,116,128,0,105,128,0,93,128,0,81,128,0,70,128,0,58,128,0,47,128,0,35,128,0,23,128,0,12,255,192,192,255,64,64,192,0,0,64,0,0,255,255,192,255,255,64,192,192,0,64,64,0,192,255,192,64,255,64,0,192,0,0,64,0,192,255,255,64,255,255,0,192,192,0,64,64,192,192,255,64,64,255,0,0,192,0,0,64,255,192,255,255,64,255,192,0,192,64,0,64,255,96,96,255,255,255,245,245,245,235,235,235,224,224,224,213,213,213,203,203,203,192,192,192,181,181,181,171,171,171,160,160,160,149,149,149,139,139,139,128,128,128,117,117,117,107,107,107,96,96,96,85,85,85,75,75,75,64,64,64,53,53,53,43,43,43,32,32,32,21,21,21,11,11,11,0,0,0
    };
    
};


MIN_EXTERNAL(ildasketch);
