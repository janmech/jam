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
#include "../jam.ilda_common/ilda_definitions.hpp"
#include "../jam.ilda_common/ilda_colors.hpp"

#define OBJECT_VERSION "jam.helios v.0.0.0"
#define HELIOS_FILE_CHUNK 1024


using namespace c74::min;

class ildaanim : public object<ildaanim>
{
    
    
public:
    
    ildaanim(const atoms& args = {}) {
        
        atom argu;
        this->_sketch_object = (c74::max::t_object*)c74::max::newinstance(symbol("jit.gl.sketch"), 0, &argu);
        
        
    }
    
    ~ildaanim() {
        c74::max::freeobject(this->_sketch_object);
    }
    
    static constexpr const char* my_description {"ildaanim"};
    
    MIN_DESCRIPTION     { "ildaanim" };
    
    MIN_TAGS            { "utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file"};
    
    inlet<> input_1    { this, "(anything) Control Messages", "anything" };
    outlet<> output_1   { this, "jit.gl.sketch draw commands" };
    
    
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
    
    message<>dictionary {
        this, "dictionary", "ilda frame dictionary",
        MIN_FUNCTION {
            
            return {};
        }
    };
    
private:
    
    c74::max::t_object *_sketch_object;
    
};


MIN_EXTERNAL(ildaanim);
