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
using fvec = std::vector<number>;
using ivec = std::vector<int>;

class ildasketch : public object<ildasketch>
{
    
private:
    
    c74::max::t_object *_sketch_object = NULL;      // Pointer to a the jit.gl.sketch object instance,
                                                    // which actually handles the rendering of the frames
    c74::max::t_object *_manager;                   // Pointer to global jam.ilda.manager object
                                                    // (stores data to be accasibele by other jam.ilda.* object)
    t_jam_im * _manager_struct_ptr = NULL;          // Pointer to max-object struct of the jam.ilda.manager object
    
    std::vector<jam::ilda::IldaFrame> _frames;
    
    float _custom_color_pallet[256] = {
        1.0 , 0.0 , 0.0 , 1.0 , 1.0 , 0.0627451 , 0.0 , 1.0 , 1.0 , 0.1254902 , 0.0 , 1.0 , 1.0 , 0.18823529 , 0.0 , 1.0 , 1.0 , 0.25098039 , 0.0 , 1.0 , 1.0 , 0.31372549 , 0.0 , 1.0 , 1.0 , 0.37647059 , 0.0 , 1.0 , 1.0 , 0.43921569 , 0.0 , 1.0 , 1.0 , 0.50196078 , 0.0 , 1.0 , 1.0 , 0.56470588 , 0.0 , 1.0 , 1.0 , 0.62745098 , 0.0 , 1.0 , 1.0 , 0.69019608 , 0.0 , 1.0 , 1.0 , 0.75294118 , 0.0 , 1.0 , 1.0 , 0.81568627 , 0.0 , 1.0 , 1.0 , 0.87843137 , 0.0 , 1.0 , 1.0 , 0.94117647 , 0.0 , 1.0 , 1.0 , 1.0 , 0.0 , 1.0 , 0.87843137 , 1.0 , 0.0 , 1.0 , 0.75294118 , 1.0 , 0.0 , 1.0 , 0.62745098 , 1.0 , 0.0 , 1.0 , 0.50196078 , 1.0 , 0.0 , 1.0 , 0.37647059 , 1.0 , 0.0 , 1.0 , 0.25098039 , 1.0 , 0.0 , 1.0 , 0.1254902 , 1.0 , 0.0 , 1.0 , 0.0 , 1.0 , 0.0 , 1.0 , 0.0 , 1.0 , 0.14117647 , 1.0 , 0.0 , 1.0 , 0.28627451 , 1.0 , 0.0 , 1.0 , 0.42745098 , 1.0 , 0.0 , 1.0 , 0.57254902 , 1.0 , 0.0 , 1.0 , 0.71372549 , 1.0 , 0.0 , 1.0 , 0.85882353 , 1.0 , 0.0 , 1.0 , 1.0 , 1.0 , 0.0 , 0.89019608 , 1.0 , 1.0 , 0.0 , 0.77647059 , 1.0 , 1.0 , 0.0 , 0.66666667 , 1.0 , 1.0 , 0.0 , 0.55686275 , 1.0 , 1.0 , 0.0 , 0.44313725 , 1.0 , 1.0 , 0.0 , 0.33333333 , 1.0 , 1.0 , 0.0 , 0.21960784 , 1.0 , 1.0 , 0.0 , 0.10980392 , 1.0 , 1.0 , 0.0 , 0.0 , 1.0 , 1.0 , 0.1254902 , 0.0 , 1.0 , 1.0 , 0.25098039 , 0.0 , 1.0 , 1.0 , 0.37647059 , 0.0 , 1.0 , 1.0 , 0.50196078 , 0.0 , 1.0 , 1.0 , 0.62745098 , 0.0 , 1.0 , 1.0 , 0.75294118 , 0.0 , 1.0 , 1.0 , 0.87843137 , 0.0 , 1.0 , 1.0 , 1.0 , 0.0 , 1.0 , 1.0 , 1.0 , 0.1254902 , 1.0 , 1.0 , 1.0 , 0.25098039 , 1.0 , 1.0 , 1.0 , 0.37647059 , 1.0 , 1.0 , 1.0 , 0.50196078 , 1.0 , 1.0 , 1.0 , 0.62745098 , 1.0 , 1.0 , 1.0 , 0.75294118 , 1.0 , 1.0 , 1.0 , 0.87843137 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 0.87843137 , 0.87843137 , 1.0 , 1.0 , 0.75294118 , 0.75294118 , 1.0 , 1.0 , 0.62745098 , 0.62745098 , 1.0 , 1.0 , 0.50196078 , 0.50196078 , 1.0 , 1.0 , 0.37647059 , 0.37647059 , 1.0 , 1.0 , 0.25098039 , 0.25098039 , 1.0 , 1.0 , 0.1254902 , 0.1254902 , 1.0 ,     };
    
    int _last_frame_index = 0;
    
        // We uase this flag to indicate whether the setter of blend/blend_mode attribute was called internally or not
        // To synchronize the displayed values they call each other and would end up in an infinite recursion otherwise
    bool _blend_setter_internal = false;
    
    dict d_outer{symbol(true)};
    dict d_inner{symbol(true)};
    
    
    
    
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
        void send(ildasketch* me) {
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
    
    c74::max::t_object * _getSketchObject() {
        if(this->_sketch_object == NULL) {
            atom argu;
            this->_sketch_object = (c74::max::t_object*)c74::max::newinstance(symbol("jit.gl.sketch"), 0, &argu);
        }
        return this->_sketch_object;
    }
    
    atom _blanking_color[4] = {atom(0.31),atom(0.31),atom(0.31), atom(0.7) };
    
    atom _monochrome_color[4] = {atom(1.),atom(1.),atom(1.), atom(1.) };
    
    int _line_width = 2;
    
    int _line_width_blanking = 1;
    
    bool _drawblanking = false;
    
    bool _force_2d = false;
    
    bool _use_custompallet = false;
    
    bool _override_opacity = false;
    
    number _opacity = false;
    
    bool _monochrome = false;
    
    void _setCustomColorByIndex(size_t color_index, number r, number g, number b, number a) {
        int offset = (int)color_index * 4;
        this->_custom_color_pallet[offset]   = r;
        this->_custom_color_pallet[offset+1] = g;
        this->_custom_color_pallet[offset+2] = b;
        this->_custom_color_pallet[offset+3] = a;
    };
    
    fvec _getCustomColorByIndex(size_t color_index) {
        color_index = (color_index > 63) ? 63 : color_index;
        fvec color_vector;
        size_t offset = color_index * 4;
        color_vector.push_back(this->_custom_color_pallet[offset]);
        color_vector.push_back(this->_custom_color_pallet[offset+1]);
        color_vector.push_back(this->_custom_color_pallet[offset+2]);
        if(!this->_override_opacity) {
            color_vector.push_back(this->_custom_color_pallet[offset+3]);
        } else {
            color_vector.push_back(this->opacity);
        }
        
        return color_vector;
    };
    
    bool _blendNameToValue(std::string mode_name, atoms *mode_values) {
        /*
         add = blend_mode 1 1
         multiply = blend_mode 2 1
         screen = blend_mode 4 1
         exclusion = blend_mode 4 5
         colorblend = blend_mode 3 4
         alphablend = blend_mode 6 7
         coloradd = blend_mode 3 1
         alphaadd = blend_mode 6 1
         */
        if(mode_name == "add") {
            mode_values->push_back(atom(1));
            mode_values->push_back(atom(1));
            return true;
        }
        
        if(mode_name == "multiply") {
            mode_values->push_back(atom(2));
            mode_values->push_back(atom(1));
            return true;
        }
        
        if(mode_name == "screen") {
            mode_values->push_back(atom(4));
            mode_values->push_back(atom(1));
            return true;
        }
        
        if(mode_name == "exclusion") {
            mode_values->push_back(atom(4));
            mode_values->push_back(atom(5));
            return true;
        }
        
        if(mode_name == "colorblend") {
            mode_values->push_back(atom(3));
            mode_values->push_back(atom(4));
            return true;
        }
        
        if(mode_name == "alphablend") {
            mode_values->push_back(atom(6));
            mode_values->push_back(atom(7));
            return true;
        }
        
        if(mode_name == "coloradd") {
            mode_values->push_back(atom(3));
            mode_values->push_back(atom(1));
            return true;
        }
        
        if(mode_name == "alphaadd") {
            mode_values->push_back(atom(6));
            mode_values->push_back(atom(1));
            return true;
        }
        
        return false;
        
        
    }
    
    std::string _blendValueToName(atoms mode_values) {
        /*
         add = blend_mode 1 1
         multiply = blend_mode 2 1
         screen = blend_mode 4 1
         exclusion = blend_mode 4 5
         colorblend = blend_mode 3 4
         alphablend = blend_mode 6 7
         coloradd = blend_mode 3 1
         alphaadd = blend_mode 6 1
         */
        if((int)mode_values[0] == 1 && (int)mode_values[1] == 1) {
            return "add";
        }
        
        if((int)mode_values[0] == 2 && (int)mode_values[1] == 1) {
            return "multiply";
        }
        
        if((int)mode_values[0] == 4 && (int)mode_values[1] == 1) {
            return "screen";
        }
        
        if((int)mode_values[0] == 4 && (int)mode_values[1] == 5) {
            return "exclusion";
        }
        
        if((int)mode_values[0] == 3 && (int)mode_values[1] == 4) {
            return "colorblend";
        }
        
        if((int)mode_values[0] == 6 && (int)mode_values[1] == 7) {
            return "alphablend";
        }
        
        if((int)mode_values[0] == 3 && (int)mode_values[1] == 1) {
            return "coloradd";
        }
        
        if((int)mode_values[0] == 6 && (int)mode_values[1] == 1) {
            return "alphaadd";
        }
        
        return "";
    }
    
public:
    
    ildasketch(const atoms& args = {}) {
        if (args.size() > 0) {
            cwarn << "Extra argumnt for oject jam.jit.gl.frame" << endl;
        }
        this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.ilda.manager"), 0, NULL);
        this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
    }
    
    ~ildasketch() {
        c74::max::freeobject(this->_getSketchObject());
        
    }
    
    
    MIN_DESCRIPTION     { "Render frames from an ILDA file to an Open GL context." };
    MIN_TAGS            { "ILDA, laser tools, utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file"};
    
    inlet<> input_1             { this, "(anything) Control Messages", "anything" };
    inlet<> input_2             { this, "(dictionary) Set the custom color pallet", "dictionary" };
    
    outlet<> outlet_dict      { this, "Dictionary describing custom color pallet", "dictionary" };
    outlet<> output_dumpout     { this, "Framecount of currently loaded ILDA file." };

    
    attribute<int> linewidth {
        this, "linewidth", 2,
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::ArgVectSize r = jam::checkAndFillAttrArgs<int>(args, &cleaned_args, 1, 1);
            if (r == jam::too_long) {
                cwarn << "missing argument for message linewidth. Assuming 1" << endl;
            }
            int val = (int)cleaned_args[0];
            val = (val < 1) ? 1 : val;
            val = (val > 10) ? 10 : val;
            cleaned_args[0] = atom(val);
            this->_line_width = cleaned_args[0];
            if(this->initialized()) {
                this->bang();
            }
            
            return cleaned_args;
        }},
        title {"Line Width"},
        description {"Line width for drawing regular segments."},
        category {"Drawing"},
    };
    
    attribute<int> linewidth_blanking {
        this, "linewidth_blanking", 1,
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::ArgVectSize r = jam::checkAndFillAttrArgs<int>(args, &cleaned_args, 1, 1);
            if (r == jam::too_long) {
                cwarn << "missing argument for message linewidth. Assuming 1" << endl;
            }
            int val = (int)cleaned_args[0];
            val = (val < 1) ? 1 : val;
            val = (val > 10) ? 10 : val;
            cleaned_args[0] = atom(val);
            this->_line_width_blanking = cleaned_args[0];
            if(this->initialized()) {
                this->bang();
            }
            
            return cleaned_args;
        }},
        title {"Line Width Blanking"},
        description {"Line width for drawing blanking segments."},
        category {"Drawing"}
    };
    
    attribute<bool> drawblanking {
        this, "drawblanking", false,
        title {"Draw Blanking"},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
                this->_drawblanking = (bool)cleaned_args[0];
                if(this->initialized()) {
                    this->bang();
                }
                return cleaned_args;
            }
        },
        description {"Draw lines where the laser is blanked. (default=0)"},
        category {"Drawing"}
    };
    
    attribute<fvec> blankingcolor {
        this, "blankingcolor", { 0.31, 0.31, 0.31, 1.},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(0,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            this->_blanking_color[0] = cleaned_args[0];
            this->_blanking_color[1] = cleaned_args[1];
            this->_blanking_color[2] = cleaned_args[2];
            this->_blanking_color[3] = cleaned_args[3];
            if(this->initialized()) {
                this->bang();
            }
            return cleaned_args;
        }},
        title {"Blanking Color"},
        description {"If draw_blanking is enabled, the color blanked lines will be drawn."},
        style {c74::min::style::color},
        category {"Drawing"}
    };
    
    attribute<bool> custompallet {
        this, "custompallet", false,
        title {"Use Custom Color Pallet"},
        description {"Use custom color pallet for frames with indexed colors."},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
                this->_use_custompallet = (bool)cleaned_args[0];
                if(this->initialized()) {
                    this->bang();
                }
                return cleaned_args;
            }
        },
        category {"Custom Color Pallet"},
        category {"Drawing"}
    };
    
    attribute<bool> overrideopacity {
        this, "overrideopacity", false,
        title {"Override Opacity"},
        description {"Override color opacity values when rendering a frame and use the value from the 'opacity' attribute instead.<br/><b>Note</b>: 'blend_enable' must be set to 1 for this to take effect."},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
                this->_override_opacity = (bool)cleaned_args[0];
                if(this->initialized()) {
                    this->bang();
                }
                return cleaned_args;
            }
        },
        category {"Custom Color Pallet"},
        category {"Drawing"}
    };
    
    attribute<number> opacity {
        this, "opacity", 1.,
        title {"Opacity"},
        description {"Opacity value applied when 'overrideopacity' is set to 1."},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 1, 1.);
                number opac_val = cleaned_args[0];
                opac_val = opac_val > 1. ? 1. : opac_val;
                opac_val = opac_val < 0. ? 0. : opac_val;
                cleaned_args[0] = opac_val;
                this->_opacity = cleaned_args[0];
                if(this->initialized()) {
                    this->bang();
                }
                return cleaned_args;
            }
        },
        category {"Custom Color Pallet"},
        category {"Drawing"}
    };
    
    attribute<bool> force_2d {
        this, "force_2d", false,
        title {"Force 2D"},
        description {"Set to 1 the z-axis position is ignored, regardles of the specified format in a frame. This can help to remove distortions in some cases, when the file is not properly generated."},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
                this->_force_2d = (bool)cleaned_args[0];
                if(this->initialized()) {
                    this->bang();
                }
                return cleaned_args;
            }
        },
        category {"Drawing"}
    };
    
    attribute<bool> monochrome {
        this, "monochrome", false,
        title {"Monochrome"},
        description {"Igore color information and draw everything  in 'monochromecolor'."},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
                this->_monochrome = (bool)cleaned_args[0];
                if(this->initialized()) {
                    this->bang();
                }
                return cleaned_args;
            }
        },
        category {"Drawing"}
    };
    
    attribute<fvec> monochromecolor {
        this, "monochromecolor", { 1., 1., 1., 1.},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(0,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            this->_monochrome_color[0] = cleaned_args[0];
            this->_monochrome_color[1] = cleaned_args[1];
            this->_monochrome_color[2] = cleaned_args[2];
            this->_monochrome_color[3] = cleaned_args[3];
            if(this->initialized()) {
                this->bang();
            }
            return cleaned_args;
        }},
        title {"Monochrome Color"},
        description {"If 'monochrome' is enabled, the color everything will be drawn."},
        style {c74::min::style::color},
        category {"Drawing"}
    };
    
    attribute<fvec> sketch_anchor {
        this, "anchor", {0., 0., 0.},
        title {"Anchor"},
        description {"The anchor position in local space (default = 0. 0. 0.). Allows for offsetting the local 3D origin around which transforms are applied."},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<float>(args, &cleaned_args, 3, 0.);
                atom sketch_atoms[3] = {cleaned_args[0], cleaned_args[1], cleaned_args[2]};
                typedmess(this->_getSketchObject(),symbol("anchor"),3,sketch_atoms);
                
                return cleaned_args;
            }
            
        },
        category {"OB3D"},
        
    };
    
    attribute<bool> sketch_antialias {
        this, "antialias", false,
        title {"Antialias"},
        description {"Antialiasing flag (default = 0) On some hardware, the blend_enable attribute must also be enabled for antialiasing to work."},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
                atom sketch_atoms[1] = {cleaned_args[0]};
                typedmess(this->_getSketchObject(),symbol("antialias"),3,sketch_atoms);
                return cleaned_args;
            }
            
        },
        category {"OB3D"},
    };
    
    attribute<bool> sketch_auto_material {
        this, "auto_material", true,
        title {"Auto Material"},
        description {"Automatic material attributes flag (default = 1) When the flag is set, and lighting is enabled for the object."},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, true);
                atom sketch_atoms[1] = {cleaned_args[0]};
                typedmess(this->_getSketchObject(),symbol("auto_material"),3,sketch_atoms);
                return cleaned_args;
            }
            
        },
        category {"OB3D"},
    };
    
    attribute<bool> sketch_automatic {
        this, "automatic", true,
        title {"Automatic"},
        description {"Automatic rendering flag (default = 1) When the flag is set, rendering occurs when the associated jit.gl.render object receives a bang message."},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, true);
                atom sketch_atoms[1] = {cleaned_args[0]};
                typedmess(this->_getSketchObject(),symbol("automatic"),1,sketch_atoms);
                return cleaned_args;
            }
            
        },
        category {"OB3D"},
    };
    
    attribute<bool> sketch_axes {
        this, "axes", false,
        title {"Axes"},
        description {"x/y/z axis rendering off/on (default = 0)"},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
                atom sketch_atoms[1] = {cleaned_args[0]};
                typedmess(this->_getSketchObject(),symbol("axes"),3,sketch_atoms);
                return cleaned_args;
            }
            
        },
        category {"OB3D"},
    };
    
    attribute<symbol> sketch_drawto {
        this, "drawto","",
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<std::string>(args, &cleaned_args, 1, "");
            atom sketch_atoms[1] = {cleaned_args[0]};
            typedmess(this->_getSketchObject(),symbol("drawto"),1,sketch_atoms);
            return cleaned_args;
        }},
        title {"Drawto"},
        description {"The named drawing context in which to draw (default = none) A named drawing context is a named instance of a jit.window, jit.pwindow, or jit.matrix object that has an instance of the jit.gl.render object associated with it."},
        category {"OB3D"}
    };
    
    attribute<bool> sketch_blend_enable {
        this, "blend_enable", false,
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
            atom sketch_atoms[1] = {cleaned_args[0]};
            typedmess(this->_getSketchObject(),symbol("blend_enable"),1,sketch_atoms);
            return cleaned_args;
        }},
        title {"Blend Enable"},
        description {"Blending flag (default = 0) When the flag is set, blending is enabled for all rendered objects."},
        category {"OB3D"}
    };
    
    attribute<fvec> sketch_position {
        this, "position", {0.0, 0.0, 0.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<float>(args, &cleaned_args, 3, 0.);
            atom sketch_atoms[3] = {args[0], args[1], args[2], };
            typedmess(this->_getSketchObject(),symbol("position"),3,sketch_atoms);
            return cleaned_args;
        }},
        title {"Position"},
        description {"The 3D origin in the form x y z (default = 0. 0. 0.)"},
        category {"OB3D"}
    };
    
    attribute<fvec> sketch_scale {
        this, "scale", {1.0, 1.0, 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<float>(args, &cleaned_args, 3, 1.);
            atom sketch_atoms[3] = {args[0], args[1], args[2], };
            typedmess(this->_getSketchObject(),symbol("scale"),3,sketch_atoms);
            return cleaned_args;
        }},
        title {"Scale"},
        description {"The 3D scaling factor in the form x y z (default = 1. 1. 1.)"},
        category {"OB3D"}
    };
    
    attribute<fvec> sketch_rotate {
        this, "rotate", {0., 0., 0., 1.},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<float>(args, &cleaned_args, 4, 0.);
            atom sketch_atoms[4] = {cleaned_args[0], cleaned_args[1], cleaned_args[2], cleaned_args[2]};
            typedmess(this->_getSketchObject(),symbol("rotate"),4,sketch_atoms);
            return cleaned_args;
        }},
        title {"Rotate"},
        description {"The angle of rotation and the xyz vector about which the rotation is performed in the form rotation-angle x y z (default = 0. 0. 0. 1.)"},
        category {"OB3D"}
    };
    
    attribute<symbol> sketch_blend {
        this, "blend", "alphablend",
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<std::string>(args, &cleaned_args, 1, "alphablend");
            std::string blend_name = (std::string)cleaned_args[0];
            if(this->initialized()) {
                if(this->_blend_setter_internal) {
                    this->_blend_setter_internal = false;
                } else {
                    atoms mess_args;
                    if(this->_blendNameToValue(blend_name, &mess_args)) {
                        this->sketch_blend_mode.set(mess_args, true);
                    }
                }
            }
            return cleaned_args;
        }},
        title {"Blend"},
        description {"The named blending mode. The possible values are:<br/>add = blend_mode 1 1<br/>multiply = blend_mode 2 1<br/>screen = blend_mode 4 1<br/>exclusion = blend_mode 4 5<br/>colorblend = blend_mode 3 4<br/>alphablend = blend_mode 6 7<br/>coloradd = blend_mode 3 1<br/>alphaadd = blend_mode 6 1<br/>"},
        range {"add", "multiply", "screen", "exclusion", "colorblend", "colorblend", "alphablend", "coloradd", "coloradd", "alphaadd"},
        category {"OB3D"}
    };
    
    attribute<ivec> sketch_blend_mode {
        this, "blend_mode", {6, 7},
        title {"Blend Mode"},
        description {"The source and destination planes associated with the blend mode (default = 6 7) Blend modes are specified in the form src_blend_mode dst_blend_mode. The supported modes are:<br/>     0 = zero<br/>     1 = one<br/>     2 = destination color<br/>     3 = source color<br/>     4 = one minus destination color<br/>     5 = one minus source color<br/>     6 = source alpha<br/>     7 = one minus source alpha<br/>     8 = destination alpha<br/>     9 = one minus destination alpha<br/>     10 = source alpha saturate"},
        setter { MIN_FUNCTION {
            
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<int>(args, &cleaned_args, 2, 0);

            for(size_t i = 0; i < cleaned_args.size(); i++) {
                cleaned_args[i] = ((int)cleaned_args[i] > 10) ? atom(10) : cleaned_args[i];
                cleaned_args[i] = ((int)cleaned_args[i] < 0) ? atom(0) : cleaned_args[i];
            }
            
            atom sketch_atoms[2] = {cleaned_args[0], cleaned_args[1]};
            typedmess(this->_getSketchObject(),symbol("blend_mode"),2,sketch_atoms);
            
            
            if(this->initialized()) {
                std::string mode_name = this->_blendValueToName(cleaned_args);
                atoms blend_args;
                blend_args.push_back(mode_name);
                this->_blend_setter_internal = true;
                this->sketch_blend.set(blend_args, true);
            }
            
            return cleaned_args;
        }},
        category {"OB3D"},
    };
    
    attribute<bool> sketch_lighting_enable {
        this, "lighting_enable", false,
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, false);
            atom sketch_atoms[1] = {cleaned_args[0]};
            typedmess(this->_getSketchObject(),symbol("lighting_enable"),1,sketch_atoms);
            return cleaned_args;
        }},
        title {"Lightning Enable"},
        description {"Lighting enabled flag (default = 0) When the flag is set, lighting is calculated."},
        category {"OB3D"}
    };
    
    
    enum class cull_face_options : int {off, back, front, two_pass, enum_count};
    enum_map cull_face_options_range = {"Off", "Back", "Front", "2Pass"};
    
    attribute<cull_face_options> sketch_cull_face {
        this, "cull_face", cull_face_options::off,cull_face_options_range,
        title {"Cull Face"},
        description {"Face culling mode (default = 0 (no culling))<br/> 0 = no culling<br/>1 = cull back face<br/>2 = cull front faces"},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<cull_face_options>(args, &cleaned_args, 1, cull_face_options::off);
            atom sketch_atoms[1] = {cleaned_args[0]};
            typedmess(this->_getSketchObject(),symbol("cull_face"),1,sketch_atoms);
            return cleaned_args;
        }},
        category {"OB3D"},
    };
    
    attribute<fvec> customcolor_0 {
        this, "customcolor_0", { 1.0 , 0.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
           
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(0,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 00"},
        description {"Custom Color for color index 0"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_1 {
        this, "customcolor_1", { 1.0 , 0.0627451 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(1,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 01"},
        description {"Custom Color for color index 1"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_2 {
        this, "customcolor_2", { 1.0 , 0.1254902 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(2,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 02"},
        description {"Custom Color for color index 2"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_3 {
        this, "customcolor_3", { 1.0 , 0.18823529 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(3,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 03"},
        description {"Custom Color for color index 3"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_4 {
        this, "customcolor_4", {1.0 , 0.25098039 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(4,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 04"},
        description {"Custom Color for color index 4"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_5 {
        this, "customcolor_5", { 1.0 , 0.31372549 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(5,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 05"},
        description {"Custom Color for color index 5"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_6 {
        this, "customcolor_6", { 1.0 , 0.37647059 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(6,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 06"},
        description {"Custom Color for color index 6"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_7 {
        this, "customcolor_7", { 1.0 , 0.43921569 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(7,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 07"},
        description {"Custom Color for color index 7"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_8 {
        this, "customcolor_8", {1.0 , 0.50196078 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(8,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 08"},
        description {"Custom Color for color index 8"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_9 {
        this, "customcolor_9", { 1.0 , 0.56470588 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(9,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 09"},
        description {"Custom Color for color index 9"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_10 {
        this, "customcolor_10", { 1.0 , 0.62745098 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(10,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 10"},
        description {"Custom Color for color index 10"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_11 {
        this, "customcolor_11", { 1.0 , 0.69019608 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(11,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 11"},
        description {"Custom Color for color index 11"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_12 {
        this, "customcolor_12", { 1.0 , 0.75294118 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(12,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 12"},
        description {"Custom Color for color index 12"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_13 {
        this, "customcolor_13", { 1.0 , 0.81568627 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(13,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 13"},
        description {"Custom Color for color index 13"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_14 {
        this, "customcolor_14", { 1.0 , 0.87843137 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(14,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 14"},
        description {"Custom Color for color index 14"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_15 {
        this, "customcolor_15", { 1.0 , 0.94117647 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(15,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 15"},
        description {"Custom Color for color index 15"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_16 {
        this, "customcolor_16", { 1.0 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(16,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 16"},
        description {"Custom Color for color index 16"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_17 {
        this, "customcolor_17", {0.87843137 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(17,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 17"},
        description {"Custom Color for color index 17"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_18 {
        this, "customcolor_18", {0.75294118 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(18,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 18"},
        description {"Custom Color for color index 18"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_19 {
        this, "customcolor_19", { 0.62745098 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(19,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 19"},
        description {"Custom Color for color index 19"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_20 {
        this, "customcolor_20", { 0.50196078 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(20,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 20"},
        description {"Custom Color for color index 20"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_21 {
        this, "customcolor_21", { 0.37647059 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(21,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 21"},
        description {"Custom Color for color index 21"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_22 {
        this, "customcolor_22", { 0.25098039 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(22,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 22"},
        description {"Custom Color for color index 22"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_23 {
        this, "customcolor_23", { 0.1254902 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(23,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 23"},
        description {"Custom Color for color index 23"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_24 {
        this, "customcolor_24", { 0.0 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(24,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 24"},
        description {"Custom Color for color index 24"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_25 {
        this, "customcolor_25", { 0.0 , 1.0 , 0.14117647 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(25,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 25"},
        description {"Custom Color for color index 25"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_26 {
        this, "customcolor_26", { 0.0 , 1.0 , 0.28627451 , 1.00},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(26,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 26"},
        description {"Custom Color for color index 26"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_27 {
        this, "customcolor_27", { 0.0 , 1.0 , 0.42745098 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(27,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 27"},
        description {"Custom Color for color index 27"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_28 {
        this, "customcolor_28", { 0.0 , 1.0 , 0.57254902 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(28,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 28"},
        description {"Custom Color for color index 28"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_29 {
        this, "customcolor_29", { 0.0 , 1.0 , 0.71372549 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(29,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 29"},
        description {"Custom Color for color index 29"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_30 {
        this, "customcolor_30", {0.0 , 1.0 , 0.85882353 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(30,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 30"},
        description {"Custom Color for color index 30"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_31 {
        this, "customcolor_31", { 0.0 , 1.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(31,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 31"},
        description {"Custom Color for color index 31"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_32 {
        this, "customcolor_32", { 0.0 , 0.89019608 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(32,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 32"},
        description {"Custom Color for color index 32"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_33 {
        this, "customcolor_33", { 0.0 , 0.77647059 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(33,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 33"},
        description {"Custom Color for color index 33"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_34 {
        this, "customcolor_34", { 0.0 , 0.66666667 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(34,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 34"},
        description {"Custom Color for color index 34"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_35 {
        this, "customcolor_35", { 0.0 , 0.55686275 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(35,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 35"},
        description {"Custom Color for color index 35"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_36 {
        this, "customcolor_36", { 0.0 , 0.44313725 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(36,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 36"},
        description {"Custom Color for color index 36"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_37 {
        this, "customcolor_37", { 0.0 , 0.33333333 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(37,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 37"},
        description {"Custom Color for color index 37"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_38 {
        this, "customcolor_38", { 0.0 , 0.21960784 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(38,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 38"},
        description {"Custom Color for color index 38"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_39 {
        this, "customcolor_39", { 0.0 , 0.10980392 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(39,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 39"},
        description {"Custom Color for color index 39"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_40 {
        this, "customcolor_40", { 0.0 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(40,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 40"},
        description {"Custom Color for color index 40"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_41 {
        this, "customcolor_41", { 0.1254902 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(41,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 41"},
        description {"Custom Color for color index 34"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_42 {
        this, "customcolor_42", {0.25098039 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(42,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 42"},
        description {"Custom Color for color index 42"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_43 {
        this, "customcolor_43", { 0.37647059 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(43,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 43"},
        description {"Custom Color for color index 43"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_44 {
        this, "customcolor_44", { 0.50196078 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(44,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 44"},
        description {"Custom Color for color index 44"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_45 {
        this, "customcolor_45", { 0.62745098 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(45,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 45"},
        description {"Custom Color for color index 45"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_46 {
        this, "customcolor_46", { 0.75294118 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(46,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 46"},
        description {"Custom Color for color index 46"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_47 {
        this, "customcolor_47", { 0.87843137 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(47,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 47"},
        description {"Custom Color for color index 47"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_48 {
        this, "customcolor_48", {1.0 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(48,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 48"},
        description {"Custom Color for color index 48"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_49 {
        this, "customcolor_49", { 1.0 , 0.1254902 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(49,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 49"},
        description {"Custom Color for color index 49"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_50 {
        this, "customcolor_50", { 1.0 , 0.25098039 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(50,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 50"},
        description {"Custom Color for color index 50"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_51 {
        this, "customcolor_51", { 1.0 , 0.37647059 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(51,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 51"},
        description {"Custom Color for color index 54"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_52 {
        this, "customcolor_52", { 1.0 , 0.50196078 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(52,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 52"},
        description {"Custom Color for color index 52"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_53 {
        this, "customcolor_53", {1.0 , 0.62745098 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(53,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 53"},
        description {"Custom Color for color index 53"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_54 {
        this, "customcolor_54", {1.0 , 0.75294118 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(54,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 54"},
        description {"Custom Color for color index 54"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_55 {
        this, "customcolor_55", { 1.0 , 0.87843137 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(55,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 55"},
        description {"Custom Color for color index 55"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_56 {
        this, "customcolor_56", { 1.0 , 1.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(56,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 56"},
        description {"Custom Color for color index 56"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_57 {
        this, "customcolor_57", { 1.0 , 0.87843137 , 0.87843137 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(57,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 57"},
        description {"Custom Color for color index 57"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_58 {
        this, "customcolor_58", { 1.0 , 0.75294118 , 0.75294118 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(58,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 58"},
        description {"Custom Color for color index 45"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_59 {
        this, "customcolor_59", { 1.0 , 0.62745098 , 0.62745098 , 1.0 },
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(59,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 59"},
        description {"Custom Color for color index 59"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_60 {
        this, "customcolor_60", { 1.0 , 0.50196078 , 0.50196078 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(60,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 60"},
        description {"Custom Color for color index 60"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_61 {
        this, "customcolor_61", { 1.0 , 0.37647059 , 0.37647059 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(61,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 61"},
        description {"Custom Color for color index 64"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_62 {
        this, "customcolor_62", { 1.0 , 0.25098039 , 0.25098039 , 1.0},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(62,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 62"},
        description {"Custom Color for color index 62"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_63 {
        this, "customcolor_63", { 1.0 , 0.1254902 , 0.1254902 , 1.00},
        setter { MIN_FUNCTION {
            atoms cleaned_args;
            jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 4, 1.);
            this->_setCustomColorByIndex(63,(number)cleaned_args[0],(number)cleaned_args[1], (number)cleaned_args[2], (number)cleaned_args[3]);
            return cleaned_args;
        }},
        title {"Custom Color 63"},
        description {"Custom Color for color index 63"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    message<>bang  {
        this, "bang", "Render last frame",
        MIN_FUNCTION {
            return this->frame(this->_last_frame_index);
        }
    };
    
    message<>reset {
        this, "reset", "reset",
        MIN_FUNCTION {
            typedmess(this->_getSketchObject(),symbol("reset"),0,0L);
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
                    //                cwarn << "no data" << endl;
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
            this->_last_frame_index = frame_index;
            if(frame_index < 0) {
                return {};
            }
            if (frame_index > this->_frames.size() -1 ) {
                return {};
            }
            
            jam::ilda::IldaFrame frame = this->_frames.at(frame_index);
            
                // FORMAT_0 : 3D Coordinates with Indexed Color
                // FORMAT_1 : 2D Coordinates with Indexed Color
                // FORMAT_2 : Color Palette
                // FORMAT_4 : 3D Coordinates with True Color
                // FORMAT_5 : 2D Coordinates with True Color
            
            int format_code = frame.getHeader().getFormatCode();
            
            if(format_code == jam::ilda::RecordFormat::FORMAT_2) {
                cwarn << "Frame " << frame_index << " contains a color pallet. Not rendering." << endl;
                return {};
            }
            
            bool is_indexed_color = (
                                     format_code == (int)jam::ilda::RecordFormat::FORMAT_0
                                     || format_code ==  (int)jam::ilda::RecordFormat::FORMAT_1
                                     );
            bool is_2d = (
                          format_code == (int)jam::ilda::RecordFormat::FORMAT_1
                          || format_code ==  (int)jam::ilda::RecordFormat::FORMAT_5
                          );
            
            typedmess(this->_getSketchObject(),symbol("reset"),0,0L);
            
            atom args_color_values[4] = {atom(1.),atom(1.),atom(1.), atom(1.)};
            atom sketch_args[5] = {atom(1), atom(1.), atom(0.), atom(0.), atom(0.) };
            
                // prior to Max 9.0.6 there is a bug causing a crash when calling "cmd_enable glcolor"
                // on jit.gl.sketch without setting the glcolor before
            typedmess(this->_getSketchObject(),symbol("glcolor"),4,sketch_args);
            
            sketch_args[0] = atom("glcolor");
            sketch_args[1] = atom(1);
            typedmess(this->_getSketchObject(),symbol("cmd_enable"),2,sketch_args);
            
            frame.reset();
            jam::ilda::IldaDataRecord data_record;
            while(frame.getNext(&data_record)) {
                bool blanking = data_record.getBlanking();
                
                if(!blanking  || this->_drawblanking) { // Do we need a color to draw?
                    number opacity = this->_override_opacity ? this->_opacity : 1.;
                    if(this->_monochrome) { // use monochrome color
                        args_color_values[0] = this->_monochrome_color[0];
                        args_color_values[1] = this->_monochrome_color[1];
                        args_color_values[2] = this->_monochrome_color[2];
                        args_color_values[3] = (this->_override_opacity) ? atom(this->_opacity) : this->_monochrome_color[3];
                    } else {
                        if(is_indexed_color) { // use indexed color
                            size_t color_index = data_record.getColorIndex();
                            fvec color_values = (this->_use_custompallet) ? this->_getCustomColorByIndex(color_index) : jam::ilda::Colors::getFloatColorByIndex(color_index, opacity);
                            args_color_values[0] = atom(color_values[0]);
                            args_color_values[1] = atom(color_values[1]);
                            args_color_values[2] = atom(color_values[2]);
                            
                            args_color_values[3] = atom( this->_override_opacity ? this->_opacity : color_values[3]);
                        } else { // use frame true color
                            uint8_t red = data_record.getRed();
                            uint8_t green = data_record.getGreen();
                            uint8_t blue = data_record.getBlue();
                            args_color_values[0] = atom((float) red / 255.);
                            args_color_values[1] = atom((float) green / 255.);
                            args_color_values[2] = atom((float) blue / 255.);
                                // ILDA RGB Colors don't have alpha, so we set it 1. if not overridden
                            args_color_values[3] = atom(opacity = this->_override_opacity ? this->_opacity : 1.);
                        }
                    }
                    
                    typedmess(this->_getSketchObject(),symbol("glcolor"),4,args_color_values);
                    
                    
                    if(blanking && this->_drawblanking) {
                        typedmess(this->_getSketchObject(),symbol("glcolor"),4,this->_blanking_color);
                    }
                }
                
                sketch_args[0] = (blanking) ? this->_line_width_blanking : this->linewidth;
                typedmess(this->_getSketchObject(),symbol("gllinewidth"),1,sketch_args);
                
                sketch_args[0] = atom(this->_normalizePosition(data_record.getPosX()));
                sketch_args[1] = atom(this->_normalizePosition(data_record.getPosY()));
                sketch_args[2] = (is_2d || this->_force_2d) ? atom(0) : atom(this->_normalizePosition(data_record.getPosY()));
                
                if(!blanking || this->_drawblanking) {
                    typedmess(this->_getSketchObject(),symbol("lineto"),3,sketch_args);
                } else {
                    typedmess(this->_getSketchObject(),symbol("moveto"),3,sketch_args);
                }
                
            }
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
            if(args.size() > 1) {
                cwarn << "extras argument for message ilda" << endl;
            }
            typedmess(this->_getSketchObject(),symbol("reset"),0,0L);
            std::string ilda_file_refence = args[0];
            std::vector<jam::ilda::IldaFrame> frames = this->_getStructPointer()->getFrames(ilda_file_refence);
            this->_frames = frames;
        
            return {};
            
        }
    };
    
    message<> exportpallet {
        this, "exportpallet", "Export the custom color pallet as dictionary",
        MIN_FUNCTION {
            
            this->d_inner.clear();
            this->d_outer.clear();
            this->d_outer["colorpallet"] = d_inner;
            c74::max::t_object* ro = (c74::max::t_object*)d_inner;
            c74::max::t_dictionary* maxdict = (c74::max::t_dictionary*)ro;
            for(size_t color_index = 0; color_index < 64; color_index++) {
                fvec color = this->_getCustomColorByIndex(color_index);
                atom color_atoms[4] = {color[0], color[1], color[2], color[3]};
                std::ostringstream index_stream;
                index_stream << color_index;
                c74::max::dictionary_appendatoms(maxdict,symbol(index_stream.str()),4,color_atoms);
            }
            
            queued_message_t msg;
            atoms msg_atoms;
            msg_atoms.push_back("dictionary");
            msg_atoms.push_back(this->d_outer.name());
            msg.set(&outlet_dict,msg_atoms);
            msg.send(this);
            
            
            return {};
            
        }
    };
    
    message<> resetpallet {
        this, "resetpallet", "Reset the custom color pallet to the ILDA default pallet colors",
        MIN_FUNCTION {
            auto attrs = this->attributes();
            atoms color_vals_atoms;
            fvec color_values_vec;
            
            for(size_t color_index = 0; color_index < 64; color_index++) {
                try {
                    color_values_vec.clear();
                    fvec color_values_vec = jam::ilda::Colors::getFloatColorByIndex(color_index);
                    std::ostringstream index_stream;
                    index_stream << color_index;
                    std::string attr_name = "customcolor_" + index_stream.str();
                    
                    attribute_base * cust_color_attr = attrs.at(attr_name);
                    color_vals_atoms.clear();
                    for(size_t val_index = 0; val_index < 4; val_index++) {
                        color_vals_atoms.push_back(color_values_vec[val_index]);
                    }
                    cust_color_attr->set(color_vals_atoms);
                } catch (std::out_of_range) {
                    continue;
                }
            }
            
            return {};
            
        }
    };
    
    message<> dictionary {
        this, "dictionary", "Use a dictionary to define the pattern of bangs produced.",
        MIN_FUNCTION {
            if(inlet != 1) {
                return {};
            }
            c74::max::t_atom d_atom = args[0];
            if(!c74::max::atomisdictionary(&d_atom)) {
                return {};
            }
            dict d {args[0]};
            
            try {
                d.at("colorpallet");
            } catch (std::runtime_error) {
                cerr << "dictionary not well formatted. Please refere to the documentaion." << endl;
                return {};
            }
            
                // Turn the atom_reference from d["colorpallet"] into an atom
            c74::min::symbol key {"colorpallet"};
            auto subdictatom = c74::min::atom(d[key].begin());
            
                // Create an unregistered subdict from the atom
            dict color_vaules_dict {subdictatom};
            
                // Generate a unique name
            auto sym = c74::min::symbol(true);
            color_vaules_dict.register_as(sym);
            atoms color_vals_atoms;
            auto attrs = this->attributes();
            for (size_t color_index = 0;  color_index < 64; color_index++) {
                    // declaring output string stream
                std::ostringstream str_stream;
                str_stream << color_index;
                std::string string_index = str_stream.str();
                try {
                    atom_reference color_values = color_vaules_dict.at(string_index);
                    long cv_count =  color_values.size();
                    if(cv_count < 4) {
                        cwarn << "missing values as index: '" << color_index << "' expecting 4 floats" << endl;
                        continue;;
                    }
                    fvec color_values_vec = static_cast<fvec>(color_values);
                    std::ostringstream index_stream;
                    index_stream << color_index;
                    std::string attr_name = "customcolor_" + index_stream.str();
                    try {
                        attribute_base * cust_color_attr = attrs.at(attr_name);
                        color_vals_atoms.clear();
                        for(size_t val_index = 0; val_index < 4; val_index++) {
                            color_vals_atoms.push_back(color_values_vec[val_index]);
                        }
                        cust_color_attr->set(color_vals_atoms);
                    } catch (std::out_of_range) {
                        continue;
                    }
                    
                } catch (std::runtime_error) {
                    continue;
                }
                
            }
            
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


MIN_EXTERNAL(ildasketch);
