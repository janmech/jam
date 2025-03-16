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
using fvec = std::vector<double>;

class ildaframe : public object<ildaframe>
{
    
private:
    
    c74::max::t_object *_sketch_object = NULL;      // Pointer to a the jit.gl.sketch object instance,
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
    
    c74::max::t_object * _getSketchObject() {
        if(this->_sketch_object == NULL) {
            atom argu;
            this->_sketch_object = (c74::max::t_object*)c74::max::newinstance(symbol("jit.gl.sketch"), 0, &argu);
        }
        return this->_sketch_object;
    }
    
    atom _blanking_color[3] = {atom(0.31),atom(0.31),atom(0.31) };
    
    int _line_width = 2;
    
    int _line_width_blanking = 1;
    
public:
    
    ildaframe(const atoms& args = {}) {
        if (args.size() > 1) {
            cout << "TEST" << endl;
            cwarn << "Extra argumnt for oject jam.jit.gl.frame" << endl;
        }
        
            // TODO: add attribute draw_to and set vale from argument
        
        this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.ilda.manager"), 0, NULL);
        this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
    }
    
    ~ildaframe() {
        c74::max::freeobject(this->_getSketchObject());
    }
    
    static constexpr const char* my_description {"ildaframe"};
    
    MIN_DESCRIPTION     { "ildaframe" };
    
    MIN_TAGS            { "utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file"};
    
    inlet<> input_1             { this, "(anything) Control Messages", "anything" };

    outlet<> output_loaded      { this, "bang when done loading file", "bang" };
    outlet<> output_dumpout     { this, "dumpout" };
    
    attribute<symbol> context {
        this, "drawto","",
        setter { MIN_FUNCTION {
            atom mess_arg = args[0];
            typedmess(this->_getSketchObject(),symbol("drawto"),1,&mess_arg);
            return args;
        }},
        title {"Drawto"},
        description {"The named drawing context in which to draw (default = none) A named drawing context is a named instance of a jit.window, jit.pwindow, or jit.matrix object that has an instance of the jit.gl.render object associated with it."},
        category {"Drawing"}
    };
    
    attribute<int, threadsafe::no, limit::clamp, allow_repetitions::no> linewidth {
        this, "linewidth", 2,
        setter { MIN_FUNCTION {
            this->_line_width = args[0];
            return args;
        }},
        title {"Line Width"},
        description {"Line width for drawing regular segments"},
        range {1, 10},
        category {"Drawing"}
    };
    
    attribute<int, threadsafe::no, limit::clamp, allow_repetitions::no> linewidth_blank {
        this, "linewidth_blank", 1,
        setter { MIN_FUNCTION {
            this->_line_width_blanking = args[0];
            return args;
        }},
        title {"Line Width Blanking"},
        description {"Line width for drawing blanking segments"},
        range {1, 10},
        category {"Drawing"}
    };
    
    attribute<fvec> position {
        this, "position", {0.0, 0.0, 0.0},
        setter { MIN_FUNCTION {
            atom mess_args[3] = {args[0], args[1], args[2], };
            typedmess(this->_getSketchObject(),symbol("position"),3,mess_args);
            return args;
        }},
        title {"Position"},
        description {"The 3D origin in the form x y z (default = 0. 0. 0.)"},
        category {"Drawing"}
    };
    
    attribute<fvec> scale {
        this, "scale", {1.0, 1.0, 1.0},
        setter { MIN_FUNCTION {
            atom mess_args[3] = {args[0], args[1], args[2], };
            typedmess(this->_getSketchObject(),symbol("scale"),3,mess_args);
            return args;
        }},
        title {"Scale"},
        description {"The 3D scaling factor in the form x y z (default = 1. 1. 1.)"},
        category {"Drawing"}
    };
    
    attribute<fvec> rotate {
        this, "rotate", {1.0, 1.0, 1.0},
        setter { MIN_FUNCTION {
            atom mess_args[3] = {args[0], args[1], args[2], };
            typedmess(this->_getSketchObject(),symbol("rotate"),3,mess_args);
            return args;
        }},
        title {"Rotate"},
        description {"The angle of rotation and the xyz vector about which the rotation is performed in the form rotation-angle x y z (default = 0. 0. 0. 1.)"},
        category {"Drawing"}
    };
    
    attribute<bool> drawblanking {
        this, "drawblanking", false,
        title {"Draw Blanking"},
        description {"Draw lines where the laser is blanked."},
        category {"Drawing"}
    };
   
    attribute<fvec> blankingcolor {
        this, "blankingcolor", { 0.31, 0.31, 0.31, 1.},
        setter { MIN_FUNCTION {
            this->_blanking_color[0] = args[0];
            this->_blanking_color[1] = args[1];
            this->_blanking_color[2] = args[1];
            return args;
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
        category {"Custom Color Pallet"},
        category {"Drawing"}
    };
    
    attribute<fvec> customcolor_0 {
        this, "customcolor_0", { 1.0 , 0.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 00"},
        description {"Custom Color for color index 0"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_1 {
        this, "customcolor_1", { 1.0 , 0.0627451 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 01"},
        description {"Custom Color for color index 1"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_2 {
        this, "customcolor_2", { 1.0 , 0.1254902 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 02"},
        description {"Custom Color for color index 2"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_3 {
        this, "customcolor_3", { 1.0 , 0.18823529 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 03"},
        description {"Custom Color for color index 3"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_4 {
        this, "customcolor_04", {1.0 , 0.25098039 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 04"},
        description {"Custom Color for color index 4"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_5 {
        this, "customcolor_5", { 1.0 , 0.31372549 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 05"},
        description {"Custom Color for color index 5"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_6 {
        this, "customcolor_6", { 1.0 , 0.37647059 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 06"},
        description {"Custom Color for color index 6"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_7 {
        this, "customcolor_7", { 1.0 , 0.43921569 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 07"},
        description {"Custom Color for color index 7"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_8 {
        this, "customcolor_8", {1.0 , 0.50196078 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 08"},
        description {"Custom Color for color index 8"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_9 {
        this, "customcolor_9", { 1.0 , 0.56470588 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 09"},
        description {"Custom Color for color index 9"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_10 {
        this, "customcolor_10", { 1.0 , 0.62745098 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 10"},
        description {"Custom Color for color index 10"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_11 {
        this, "customcolor_11", { 1.0 , 0.69019608 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 11"},
        description {"Custom Color for color index 11"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_12 {
        this, "customcolor_12", { 1.0 , 0.75294118 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 12"},
        description {"Custom Color for color index 12"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_13 {
        this, "customcolor_13", { 1.0 , 0.81568627 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 13"},
        description {"Custom Color for color index 13"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_14 {
        this, "customcolor_14", { 1.0 , 0.87843137 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 14"},
        description {"Custom Color for color index 14"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_15 {
        this, "customcolor_15", { 1.0 , 0.94117647 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 15"},
        description {"Custom Color for color index 15"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_16 {
        this, "customcolor_16", { 1.0 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 16"},
        description {"Custom Color for color index 16"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_17 {
        this, "customcolor_17", {0.87843137 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 17"},
        description {"Custom Color for color index 17"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_18 {
        this, "customcolor_18", {0.75294118 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 18"},
        description {"Custom Color for color index 18"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_19 {
        this, "customcolor_19", { 0.62745098 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 19"},
        description {"Custom Color for color index 19"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_20 {
        this, "customcolor_20", { 0.50196078 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 20"},
        description {"Custom Color for color index 20"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_21 {
        this, "customcolor_21", { 0.37647059 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 21"},
        description {"Custom Color for color index 21"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_22 {
        this, "customcolor_22", { 0.25098039 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 22"},
        description {"Custom Color for color index 22"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_23 {
        this, "customcolor_23", { 0.1254902 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 23"},
        description {"Custom Color for color index 23"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_24 {
        this, "customcolor_24", { 0.0 , 1.0 , 0.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 24"},
        description {"Custom Color for color index 24"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_25 {
        this, "customcolor_25", { 0.0 , 1.0 , 0.14117647 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 25"},
        description {"Custom Color for color index 25"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_26 {
        this, "customcolor_26", { 0.0 , 1.0 , 0.28627451 , 1.00},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 26"},
        description {"Custom Color for color index 26"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_27 {
        this, "customcolor_27", { 0.0 , 1.0 , 0.42745098 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 27"},
        description {"Custom Color for color index 27"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_28 {
        this, "customcolor_28", { 0.0 , 1.0 , 0.57254902 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 28"},
        description {"Custom Color for color index 28"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_29 {
        this, "customcolor_29", { 0.0 , 1.0 , 0.71372549 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 29"},
        description {"Custom Color for color index 29"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_30 {
        this, "customcolor_30", {0.0 , 1.0 , 0.85882353 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 30"},
        description {"Custom Color for color index 30"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_31 {
        this, "customcolor_31", { 0.0 , 1.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 31"},
        description {"Custom Color for color index 31"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_32 {
        this, "customcolor_32", { 0.0 , 0.89019608 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 32"},
        description {"Custom Color for color index 32"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_33 {
        this, "customcolor_33", { 0.0 , 0.77647059 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 33"},
        description {"Custom Color for color index 33"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_34 {
        this, "customcolor_34", { 0.0 , 0.66666667 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 34"},
        description {"Custom Color for color index 34"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_35 {
        this, "customcolor_35", { 0.0 , 0.55686275 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 35"},
        description {"Custom Color for color index 35"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_36 {
        this, "customcolor_36", { 0.0 , 0.44313725 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 36"},
        description {"Custom Color for color index 36"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_37 {
        this, "customcolor_37", { 0.0 , 0.33333333 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 37"},
        description {"Custom Color for color index 37"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_38 {
        this, "customcolor_38", { 0.0 , 0.21960784 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 38"},
        description {"Custom Color for color index 38"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_39 {
        this, "customcolor_39", { 0.0 , 0.10980392 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 39"},
        description {"Custom Color for color index 39"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_40 {
        this, "customcolor_40", { 0.0 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 40"},
        description {"Custom Color for color index 40"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_41 {
        this, "customcolor_41", { 0.1254902 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 41"},
        description {"Custom Color for color index 34"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_42 {
        this, "customcolor_42", {0.25098039 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 42"},
        description {"Custom Color for color index 42"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_43 {
        this, "customcolor_43", { 0.37647059 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 43"},
        description {"Custom Color for color index 43"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_44 {
        this, "customcolor_44", { 0.50196078 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 44"},
        description {"Custom Color for color index 44"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_45 {
        this, "customcolor_45", { 0.62745098 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 45"},
        description {"Custom Color for color index 45"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_46 {
        this, "customcolor_46", { 0.75294118 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 46"},
        description {"Custom Color for color index 46"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_47 {
        this, "customcolor_47", { 0.87843137 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 47"},
        description {"Custom Color for color index 47"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_48 {
        this, "customcolor_48", {1.0 , 0.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 48"},
        description {"Custom Color for color index 48"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_49 {
        this, "customcolor_49", { 1.0 , 0.1254902 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 49"},
        description {"Custom Color for color index 49"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_50 {
        this, "customcolor_50", { 1.0 , 0.25098039 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 50"},
        description {"Custom Color for color index 50"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_51 {
        this, "customcolor_51", { 1.0 , 0.37647059 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 51"},
        description {"Custom Color for color index 54"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_52 {
        this, "customcolor_52", { 1.0 , 0.50196078 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 52"},
        description {"Custom Color for color index 52"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_53 {
        this, "customcolor_53", {1.0 , 0.62745098 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 53"},
        description {"Custom Color for color index 53"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_54 {
        this, "customcolor_54", {1.0 , 0.75294118 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 54"},
        description {"Custom Color for color index 54"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_55 {
        this, "customcolor_55", { 1.0 , 0.87843137 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 55"},
        description {"Custom Color for color index 55"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_56 {
        this, "customcolor_56", { 1.0 , 1.0 , 1.0 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 56"},
        description {"Custom Color for color index 56"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_57 {
        this, "customcolor_57", { 1.0 , 0.87843137 , 0.87843137 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 57"},
        description {"Custom Color for color index 57"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_58 {
        this, "customcolor_58", { 1.0 , 0.75294118 , 0.75294118 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 58"},
        description {"Custom Color for color index 45"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_59 {
        this, "customcolor_59", { 1.0 , 0.62745098 , 0.62745098 , 1.0 },
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 59"},
        description {"Custom Color for color index 59"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_60 {
        this, "customcolor_60", { 1.0 , 0.50196078 , 0.50196078 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 60"},
        description {"Custom Color for color index 60"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_61 {
        this, "customcolor_61", { 1.0 , 0.37647059 , 0.37647059 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 61"},
        description {"Custom Color for color index 64"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_62 {
        this, "customcolor_62", { 1.0 , 0.25098039 , 0.25098039 , 1.0},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 62"},
        description {"Custom Color for color index 62"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    attribute<fvec> customcolor_63 {
        this, "customcolor_63", { 1.0 , 0.1254902 , 0.1254902 , 1.00},
        setter { MIN_FUNCTION {
            return args;
        }},
        title {"Custom Color 63"},
        description {"Custom Color for color index 63"},
        style {c74::min::style::color},
        category {"Custom Color Pallet"}
    };
    
    message<>bang  {
        this, "bang", "trigger output",
        MIN_FUNCTION {
            // anim aaa
            atom msg_args[1];
            msg_args[0] = atom("aaa");
            typedmess(this->_getSketchObject(),symbol("foo"),0,0L);
            typedmess(this->_getSketchObject(),symbol("anim"),1,msg_args);
            if(this->_frames.size() == 0) {
                cwarn << "no data" << endl;
                return {};
            }
            
            return {};
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
            typedmess(this->_getSketchObject(),symbol("reset"),0,0L);
            
            atom args_color_values[3] = {atom(1.),atom(1.),atom(1.)};
            atom sketch_args[5] = {atom(1), atom(1.), atom(0.), atom(0.), atom(0.) };
            
            typedmess(this->_getSketchObject(),symbol("glcolor"),4,sketch_args);
            
            sketch_args[0] = atom("glcolor");
            sketch_args[1] = atom(1);
            typedmess(this->_getSketchObject(),symbol("cmd_enable"),2,sketch_args);
            
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
                typedmess(this->_getSketchObject(),symbol("glcolor"),3,args_color_values);
                
                bool blanking = data_record.getBlanking();
                if(blanking && drawblanking) {
                    typedmess(this->_getSketchObject(),symbol("glcolor"),3,this->_blanking_color);
                }
                sketch_args[0] = (blanking) ? this->_line_width_blanking : this->linewidth;
                typedmess(this->_getSketchObject(),symbol("gllinewidth"),1,sketch_args);
                
                int pos_x = data_record.getPosX();
                int pos_y = data_record.getPosY();
                sketch_args[0] = atom(this->_normalizePosition(pos_x));
                sketch_args[1] = atom(this->_normalizePosition(pos_y));
                sketch_args[2] = atom(0); // For now we are ignoring z axis
                
                if(!blanking || drawblanking) {
                    typedmess(this->_getSketchObject(),symbol("lineto"),3,sketch_args);
                } else {
                    typedmess(this->_getSketchObject(),symbol("moveto"),3,sketch_args);
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
