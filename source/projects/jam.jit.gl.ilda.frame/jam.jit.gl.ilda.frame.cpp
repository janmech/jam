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

#define OBJECT_VERSION "jam.helios v.0.0.0"
#define HELIOS_FILE_CHUNK 1024


using namespace c74::min;

class ildaframe : public object<ildaframe>
{
    
    
public:
    
    ildaframe(const atoms& args = {}) {
        if (args.size() > 1) {
            cout << "TEST" << endl;
            cwarn << "Extra argumnt for oject jam.jit.gl.frame" << endl;
        }
        
            // TODO: add attribute draw_to and set vale from argument
        
        
        atom argu;
        this->_sketch_object = (c74::max::t_object*)c74::max::newinstance(symbol("jit.gl.sketch"), 0, &argu);
        
        
    }
    
    ~ildaframe() {
        c74::max::freeobject(this->_sketch_object);
    }
    
    static constexpr const char* my_description {"ildaframe"};
    
    MIN_DESCRIPTION     { "ildaframe" };
    
    MIN_TAGS            { "utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file"};
    
    inlet<> input_1    { this, "(anything) Control Messages", "anything" };
    outlet<> output_1   { this, "frame count" };
    outlet<> output_2   { this, "current frame", "dictionary" };
    
    message<>bang  {
        this, "bang", "trigger output",
        MIN_FUNCTION {
            cout << "bang ilda frame dictionary" << endl;
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
    
    message<> frame {
        this, "frame", "render frame",
        MIN_FUNCTION {
            
//            output_1("dictionary", this->_frames_dict.name());
            return {};
        }
    };
    
    message<>dictionary {
        this, "dictionary", "ilda frame dictionary",
        MIN_FUNCTION {
            dict incoming_dict = {args[0]};
            this->_current_frame_dict.clear();
            this->_current_frame_dict.copyunique(incoming_dict);
            
            int data_record_count = 0;
            try {
                    // reading data_record_cout
                std::string count_string = atom(this->_current_frame_dict.at("data_record_count"));
                data_record_count = std::atoi(count_string.c_str());
            } catch(std::exception& ex) {
                
                cerr << "1. error parsing frame dict." << endl;
                cerr << ex.what() << endl;
                return {};
            }
            
                // getting sub dict data_recods
            c74::min::symbol key_data_records {"data_records"};
            auto data_records_dict_atom = c74::min::atom(this->_current_frame_dict[key_data_records].begin());
                // Create an unregistered subdict from the atom
            dict data_records_dict {data_records_dict_atom};
            
            try {
                atom format_code = this->_current_frame_dict.at("format_code");
                int v_format_code = (int)format_code;
                bool is_indexed_color = (
                                         v_format_code == (int)jam::ilda::RecordFormat::FORMAT_0
                                         || v_format_code ==  (int)jam::ilda::RecordFormat::FORMAT_1
                                         );
                    typedmess(this->_sketch_object,symbol("reset"),0,0L);
                
                atom args_color_values[3] = {atom(1.),atom(1.),atom(1.)};
                atom args[5] = {atom(1), atom(1.), atom(0.), atom(0.), atom(0.) };
                
                    typedmess(this->_sketch_object,symbol("glcolor"),4,args);
                
                args[0] = atom("glcolor");
                args[1] = atom(1);
                    typedmess(this->_sketch_object,symbol("cmd_enable"),2,args);
                
                args[0] = atom(2.);
                    typedmess(this->_sketch_object,symbol("gllinewidth"),1,args);
                
                for(int i = 0; i < (int)data_record_count; i++) {
                        // geting sub dict data_record
                    c74::min::symbol key_record_index {i};
                    auto data_record_dict_atom = c74::min::atom(data_records_dict[key_record_index].begin());
                    dict data_record_dict {data_record_dict_atom};
                    
                    try {
                        if(is_indexed_color) {
                            atom a_color_index = data_record_dict.at("color_index");
                            int v_color_idex = (int)a_color_index;
                            std::vector<float> color_values = jam::ilda::Colors::getFloatColorByIndex((uint8_t)v_color_idex);
                            
                            args_color_values[0] = atom(color_values[0]);
                            args_color_values[1] = atom(color_values[1]);
                            args_color_values[2] = atom(color_values[2]);
                        } else {
                            atom a_red = data_record_dict.at("red");
                            atom a_green = data_record_dict.at("green");
                            atom a_blue = data_record_dict.at("blue");
                            int v_red = a_red;
                            int v_green = a_green;
                            int v_blue = a_blue;
                            args_color_values[0] = atom((float) v_red / 255.);
                            args_color_values[1] = atom((float) v_green / 255.);
                            args_color_values[2] = atom((float) v_blue / 255.);
                        }
                        
                        atom blanking = data_record_dict.at("blanking");
                        bool value_blanking = (bool)blanking;
                        atom a_pos_x = data_record_dict.at("pos_x");
                        atom a_pos_y = data_record_dict.at("pos_y");
                        std::string s_pos_x = a_pos_x;
                        std::string s_pos_y = a_pos_y;
                        int v_pos_x = std::atoi(s_pos_x.c_str());
                        int v_pos_y = std::atoi(s_pos_y.c_str());
                        args[0] = atom(this->_normalizePosition(v_pos_x));
                        args[1] = atom(this->_normalizePosition(v_pos_y));
                        args[2] = atom(0); // For now we are ignoring z axis
                        
                        if(value_blanking) {
                                typedmess(this->_sketch_object,symbol("moveto"),3,args);
                            
                        } else {
                                typedmess(this->_sketch_object,symbol("glcolor"),3,args_color_values);
                                typedmess(this->_sketch_object,symbol("lineto"),3,args);
                        }
                        
                    } catch(...) {
                        cwarn << "skipped frame " << i << endl;
                        continue;
                    }
                    
                }
            } catch (std::exception& ex) {
                cerr << "2. error parsing frame dict." << endl;
                cerr << ex.what() << endl;
            }
            return {};
        }
    };
    
private:
    
    c74::max::t_object *_sketch_object;
    dict _frames_dict{symbol(true)};
    dict _current_frame_dict{symbol(true)};
    long _frame_count = 0;
    float _normalizePosition(int pos) {
        pos = (pos < -32768) ? -32768 : pos;
        pos = (pos > 32767) ? 32767 : pos;
        if (pos > 0) {
            return ((float)pos / 32767.) * 0.85;
        }
        return ((float)pos / 32768) * 0.85;
    };
};


MIN_EXTERNAL(ildaframe);


/*
 long getentrycount() {
 return (long)max::dictionary_getentrycount(m_instance);
 }
 
 std::vector<symbol> getkeys() {
 std::vector<symbol>k;
 max::t_symbol    **keys = NULL;
 long        numkeys = 0;
 long        i;
 
 max::dictionary_getkeys(m_instance, &numkeys, &keys);
 for(i=0; i<numkeys; i++){
 k.push_back(symbol(keys[i]));
 }
 if(keys) {
 max::dictionary_freekeys(m_instance, numkeys, keys);
 }
 return k;
 }
 */
