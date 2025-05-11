    /// @file
    ///	@ingroup    jam
    ///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
    ///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#define NANOSVG_IMPLEMENTATION
#define STB_TRUETYPE_IMPLEMENTATION

#include <iostream>
#include <string>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <queue>
#include <string>
#include <CoreText/CoreText.h>
#include <CoreFoundation/CoreFoundation.h>
#include "c74_min.h"
#include "jam.shape.hpp"
#include "../jam.helper/attribute_args_helper.hpp"
#include "jam_compose_data.hpp"
#include "../jam.ilda_common/ilda_frame.hpp"
#include "../jam.ilda_common/ilda_header.hpp"
#include "../jam.ilda_common/ilda_data_record.hpp"
#include "../jam.ilda_common/ilda_definitions.hpp"
#include "../jam.ilda.manager/jam.ilda.manager.hpp"
#include "../jam.ilda_common/ilda_file_processor.hpp"
#include "../jam.ilda_common/ilda_colors.hpp"
#include "../jam.ilda_common/ttf_file_processor.hpp"
#include "../jam.ilda_common/nanosvg.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define BINARY_FILE_CHUNK 1024


using namespace c74::min;
using namespace jam::compose;
using Point2D = jam::Point2D;
using VecPoint2D = std::vector<jam::Point2D>;
using VecDataPoints = std::vector<DataPoint>;
using VecDataSets = std::vector<DataSet>;
using VecGlyphPoints = std::vector<jam::ttf::GlyphVertex>;



class ildacompose : public object<ildacompose>
{

protected:
    
    std::string _instance_id = "";                  // Unique ID for each object instance.
                                                    // Used to itentify loaded ILDA filed data in the global jam.ilda.manager
    c74::max::t_object *_manager;                   // Pointer to global jam.ilda.manager object
                                                    // (stores data to be accasibele by other jam.ilda.* object)
    t_jam_im * _manager_struct_ptr = NULL;          // Pointer to max-object struct of the jam.ilda.manager object
    
    atoms _import_args;                             // Stores arguments of import message,
                                                    // to be accasible in the scope of the file loader thread
    c74::max::t_filehandle _file_handle;            // File handle for importing SVG files.
    
    char _filename[c74::max::MAX_PATH_CHARS] = {0}; // File name of ILDA file toi be imported
    
    std::vector<jam::Shape> _svg_shapes;            // Vector of Shapes from parsed SVG file
    
    std::string _company_name = "NOT_SET";          // Company name set to frame headers
    
    std::string _frame_name_prefix = "";            // Prefix for frame name set to frame headers
    
    size_t _edit_frame = 0;                         // Frame with this index is selected for editing
    
    jam::ilda::IldaFileProcessor _ildaFileProcessor; // Class with functions for ILDA file processing/parsing
    
    jam::ttf::TtfFileProcessor _ttfFileProcessor;    // Class with function for TrueType font rendering
    
    struct FontInfo {
        std::string file_path;
        int face_index;
    };
    
    std::map<std::string, FontInfo>_available_fonts;
    
        /// Struct to encapsulate sending messages to outlets via the timer - for thread safty
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
        void send(ildacompose* me) {
            me->_enqueue_msg_to_max(*this);
            me->deliverer_to_max.delay(0);
        }
    } queued_message_t;
    
    typedef struct RgbColor {
        number r = 1.;
        number g = 1.;
        number b = 1.;
    } rgb_color_t;
    
        /// Drawing Color
    rgb_color_t _color;
    
        /// Pen Position for writing text
    jam::ttf::Point2D _pen_pos = { 0., 0.};
    
        /// Apply kerning to text rendering
    bool _kerning = true;
    
    number _font_size = 20;
    
        /// FIFO queue for messages to be sent to outlets
    fifo<queued_message_t> _to_max_queue { 10000 };
    
        /// Mutex lock for outlet message thread safty
    std::mutex _enqueue_msg_lock;
    
    std::mutex _frame_vector_lock;
    
    bool _is_parsing_svg = false;
    
        /// Vector of IldaFrames currenly available
    std::vector<jam::ilda::IldaFrame> _ilda_frames;     // ILDA frames rendered from data sets
    
    std::vector<DataSet> _data_sets;                    // Data sets contain ilda frame information before scale/rotation precessing
    
    void _updateFonts() {
        _available_fonts.clear();

        CTFontCollectionRef collection = CTFontCollectionCreateFromAvailableFonts(nullptr);
        if (!collection) return;

        CFArrayRef descriptors = CTFontCollectionCreateMatchingFontDescriptors(collection);
        if (!descriptors) {
            CFRelease(collection);
            return;
        }
        
        CFStringRef font_index_attribute = CFSTR("NSCTFontIndexAttribute");

        CFIndex count = CFArrayGetCount(descriptors);
        for (CFIndex i = 0; i < count; ++i) {
            CTFontDescriptorRef desc = (CTFontDescriptorRef)CFArrayGetValueAtIndex(descriptors, i);

            // Get font file URL
            CFURLRef url_ref = (CFURLRef)CTFontDescriptorCopyAttribute(desc, kCTFontURLAttribute);
            if (!url_ref) continue;

            char path[PATH_MAX];
            if (!CFURLGetFileSystemRepresentation(url_ref, true, (UInt8*)path, sizeof(path))) {
                CFRelease(url_ref);
                continue;
            }
            std::string path_str(path);

            // Get the face index (important for TTC files)
            int face_index = 0;
            CFNumberRef index_ref = (CFNumberRef)CTFontDescriptorCopyAttribute(desc, font_index_attribute);
            if (index_ref) {
                CFNumberGetValue(index_ref, kCFNumberIntType, &face_index);
                CFRelease(index_ref);
            }

            // Get the display name
            CFStringRef name_ref = (CFStringRef)CTFontDescriptorCopyAttribute(desc, kCTFontDisplayNameAttribute);
            char name[256] = "Unknown";
            if (name_ref) {
                CFStringGetCString(name_ref, name, sizeof(name), kCFStringEncodingUTF8);
                CFRelease(name_ref);
            }

            std::string display_Name(name);

            // filter by file extension. We support only TTF and TTC.
            std::string ext = path_str.substr(path_str.find_last_of('.') + 1);
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (ext == "ttf" || ext == "ttc") {
                _available_fonts[display_Name] = FontInfo{path_str, face_index};
            }

            CFRelease(url_ref);
        }

        CFRelease(descriptors);
        CFRelease(collection);
    }
    
        /// Set if the instance currently in the process of importing a file
    void _setSvgParsingState(bool state) {
        if(state != this->_is_parsing_svg) {
            this->_is_parsing_svg = state;
        }
    }
    
        /// get if the nstance currently in the process of importing a file
    bool _getSvgParsingState() {
        return this->_is_parsing_svg;
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
    
        /// deueueing queued outlet messages (thread safe)
    bool _dequeue_msg_to_max(queued_message_t &msg_data) {
        _enqueue_msg_lock.lock();
        bool result = this->_to_max_queue.try_dequeue(msg_data);
        _enqueue_msg_lock.unlock();
        return result;
    }
    
       /// translate from normalized coordinates (-1. to 1.) to ILDA file coordinates
    int _deNormalizePosition(number pos) {
        int de_normalized = static_cast<int>(pos * 32000);
        return std::clamp(de_normalized, -32767, 32767);
            //        if(pos < 0) {
            //            return static_cast<int>(pos * 32768);
            //        }
            //        return static_cast<int>(pos * 32767);
    };
    
        /// adjust the current edit frame index when frame count has changed, to make sure it doen't go out of bounds
    void _updateEditFrame() {
        if(this->_ilda_frames.size() == 0) {
            this->_edit_frame = 0;
        } else {
            if(this->_edit_frame > this->_ilda_frames.size() - 1) {
                this->_edit_frame = this->_ilda_frames.size() - 1;
            }
        }
    }
    
        /// sends out current edit frame, frame count and the file reference
    void _updateOutlets() {
        this->_updateEditFrame();
        atoms msg_atoms;
        queued_message_t msg;
        
        msg_atoms.clear();
        msg_atoms.push_back(this->_ilda_frames.size());
        msg.set(&o_framecount, msg_atoms);
        msg.send(this);
        
        msg_atoms.clear();
        msg_atoms.push_back(this->_edit_frame);
        msg.set(&o_edit_frame, msg_atoms);
        msg.send(this);
        
        msg_atoms.clear();
        msg_atoms.push_back("ilda");
        msg_atoms.push_back(this->_instance_id);
        msg.set(&o_file_reference, msg_atoms);
        msg.send(this);
    }
    
      /// add an empty frame at the end
    void _appendEmptyFrame() {
        this->_frame_vector_lock.lock();
        // add ILDA frame
        jam::ilda::IldaFrame f;
        jam::ilda::IldaHeader h;
        h.setFormatCode(jam::ilda::RecordFormat::FORMAT_5);
        h.setIsColorPallet(false);
        h.setDataRecordCount(0);
        f.setHeader(h);
        this->_ilda_frames.push_back(f);
        this->_updateFrameHeaders();
        this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
        this->_edit_frame = this->_ilda_frames.size() - 1;
        
        // add raw frame
        DataSet raw_frame;
        this->_data_sets.push_back(raw_frame);
        this->_frame_vector_lock.unlock();
        
        this->_updateOutlets();
    }
    
    void _removeEmptyFrames() {
        for (auto it = this->_ilda_frames.begin(); it != this->_ilda_frames.end();) {
            auto f = *it;
            if(f.getHeader().getDataRecordCount() == 0) {
                it = this->_ilda_frames.erase(it);
            } else {
                it++;
            }
        }
        for (auto it = this->_data_sets.begin(); it != this->_data_sets.end();) {
            auto ds = *it;
            if(ds.getDataPoints().size() == 0) {
                it = this->_data_sets.erase(it);
            } else {
                it++;
            }
        }
    }
    
        /// update frames in sequens and frame number for all frames
    void _updateFrameHeaders() {
        size_t frame_count = this->_ilda_frames.size();
        for(size_t i = 0; i < frame_count; i++) {
            this->_ilda_frames[i].getHeader().setFramesInSequence(frame_count);
            this->_ilda_frames[i].getHeader().setFrameNumber(i);
            this->_ilda_frames[i].getHeader().setCompanyName(this->_company_name);
            this->_ilda_frames[i].getHeader().setFrameName(this->_makeFrameName(static_cast<int>(i)));
            this->_ilda_frames[i].getHeader().setDataRecordCount(this->_ilda_frames[i].getDataRecordCount());
        }
    }
    
    void _parseFrames2DToTrueColor(std::vector<jam::ilda::IldaFrame> &frames) {
        jam::ilda::Colors *col = new jam::ilda::Colors();
        
        for(size_t i = 0; i < frames.size(); i++) {
            jam::ilda::IldaHeader h = frames[i].getHeader();
            jam::ilda::RecordFormat rec_format = h.getFormatCode();
            
            if(rec_format == jam::ilda::RecordFormat::FORMAT_2) { // we ignore color palette frames
                continue;
            }
            h.setFormatCode(jam::ilda::RecordFormat::FORMAT_4); // 2D True Color
            frames[i].setHeader(h);
            std::vector<jam::ilda::IldaDataRecord> dr = frames[i].getDataRecords();
            for(size_t j = 0; j < dr.size(); j++) {
                uint8_t color_index = dr[j].getColorIndex();
                std::vector<number> col_vals = col->getFloatColorByIndex((size_t)color_index);
                dr[j].setRed((uint8_t)(col_vals[0] * 255.));
                dr[j].setGreen((uint8_t)(col_vals[1] * 255.));
                dr[j].setBlue((uint8_t)(col_vals[2] * 255.));
                dr[j].setPosZ(0);
            }
            frames[i].setDataRecords(dr);
            
        }
        delete col;
    }
    
        /// Formats string to match the requirements of the ILDA frame header: Non ASCII characters are subtituted with _
        /// @param  s                    string reference to the input name
        /// @param  max_len     maximum output string length
    void _formatIldaString(std::string & s, size_t max_len = 8) {
        std::string formatted = "";
        if (s.size() > max_len) {
            s.resize(max_len);
        }
        
        size_t i = 0;
        while (i < s.size()) {
            unsigned char c = static_cast<unsigned char>(s[i]);
            
            if (c < 128) {
                    // ASCII character, copy as-is
                formatted += c;
                i += 1;
            } else {
                    // Start of a multi-byte UTF-8 character
                if ((c >> 5) == 0b110) i += 2;       // 2-byte sequence
                else if ((c >> 4) == 0b1110) i += 3; // 3-byte sequence
                else if ((c >> 3) == 0b11110) i += 4; // 4-byte sequence
                else i += 1; // Invalid byte, skip it anyway
                
                formatted += '_'; // Replace the whole character with one underscore
            }
        }
        s = formatted;
    }
    
        /// generate a string for the frame name in the frame header based on the frameprefix attribute and the frame index
        /// @param   frame_index        index of the frame
    std::string _makeFrameName(int frame_index) {
        std::string name = this->_frame_name_prefix;
        int width = static_cast<int>(8 - this->_frame_name_prefix.size());
        std::ostringstream ss;
        ss << std::setw(width) << std::setfill('0') << frame_index;
        std::string s2(ss.str());
        std::cout << s2;
        name = name + s2;
        return name;
    };
    
        /// generate point for ellipes
        /// @param   c_x                               center coordinate x
        /// @param   c_y                               center coordinate y
        /// @param   rx                                 radius x
        /// @param   ry                                 radius y
        /// @param   theta_start            start angle in degrees (0º - 360º)
        /// @param   theta_end                 end angle in degrees (0º - 360º)
        /// @param   segments                   number of line segments
    VecDataPoints _makeEllipse(
                            DataPoint c,
                            DataPoint r,
                            const number theta_start = 0,
                            const number theta_end = 360,
                            int segments = 50
                            ) {
        VecDataPoints points;
        if(theta_start == theta_end) {
            return points;
        }
        number rad_start = theta_start * (PI / 180.);
        number rad_end = theta_end * (PI / 180.);
        number rad_range = rad_end - rad_start;
        
        
        for (int i = 0; i <= segments; ++i) {
            DataPoint p;
            number angle = rad_start + (rad_range * i / segments);
            // number angle = rad_start + (number)(rad_range * (number)i / (number)segments);
            p.x = c.x + r.x * std::cos(angle);
            p.y = c.y + r.y * std::sin(angle);
            points.push_back(p);
        }
        return points;
        
    }
    
    void _addDataPointsToEditDataSet(VecDataPoints points) {
        for(size_t i = 0; i < points.size(); i++) {
            bool is_first = (i == 0);
            DataPoint dp;
            dp.x = points[i].x;
            dp.y = points[i].y;
            dp.r = (is_first) ? 0. : this->_color.r;
            dp.g = (is_first) ? 0. : this->_color.g;
            dp.b = (is_first) ? 0. : this->_color.b;
            dp.blanking = is_first;
            this->_data_sets[this->_edit_frame].addDataPoint(dp);
        }
    }
    
        /// generate point for a rectange with rounded corners
        /// @param   tl                               top left coordinates of the rectange
        /// @param   br                               bottom right coordinates of the rectange
        /// @param   rnd                             corner roundes (0. - 1.) the higer the number the greater the radius of the corner arc
        /// @param   segments                  number of line segments
    VecDataPoints _makeRectangle(
                              DataPoint tl,
                              DataPoint br,
                              number rnd, // corner roundness
                              int segments = 10
                              ) {
        
        VecDataPoints points;
        DataPoint tr = {br.x, tl.y};
        DataPoint bl = {tl.x, br.y};
        if(rnd == 0.) {
            points.push_back(tl);
            points.push_back(tr);
            points.push_back(br);
            points.push_back(bl);
            points.push_back(tl);
            return points;
        }
            // calculate radius as fraction of shorter rectangle side.
            // the rnd parameter describes the roundness of a corner
            // 0: no rounding, 1: max rounding
            // 1 means we calculaten an arc with the radius on 1/2 of the shorter rectangle side.
        number length_horizontal = abs(tr.x - tl.x);
        number length_vertical = abs(tl.y - bl.y);
        number min_lenght = fmin(length_horizontal, length_vertical);
        number radius = min_lenght * rnd / 2.;
        
            // arc circle radius
        DataPoint circle_r = {radius, radius};
        
            // arc top left
        DataPoint arc_center_tl = {tl.x + radius, tl.y - radius};
        VecDataPoints arc_tl = this->_makeEllipse(arc_center_tl, circle_r, 180, 90, segments);
        
            // arc top right
        DataPoint arc_center_tr = {tr.x - radius, tr.y - radius};
        VecDataPoints arc_tr = this->_makeEllipse(arc_center_tr, circle_r, 90, 0, segments);
        
            // arc bottom right
        DataPoint arc_center_br = {br.x - radius, br.y + radius};
        VecDataPoints arc_br = this->_makeEllipse(arc_center_br, circle_r, 0, -90, segments);
        
            // arc bottom left
        DataPoint arc_center_bl = {bl.x + radius, bl.y + radius};
        VecDataPoints arc_bl = this->_makeEllipse(arc_center_bl, circle_r, 270, 180, segments);
        
        points.insert(points.end(), arc_tl.begin(), arc_tl.end());
        points.insert(points.end(), arc_tr.begin(), arc_tr.end());
        points.insert(points.end(), arc_br.begin(), arc_br.end());
        points.insert(points.end(), arc_bl.begin(), arc_bl.end());
        points.push_back(arc_tl[0]);
        
        return points;
    }
    
        /// generate point a cubic bezier curve
        /// @param   p0                               start point
        /// @param   p01                             first control point
        /// @param   p02                             second control point
        /// @param   p03                             end point
        /// @param   segments                  number of line segments
        /// @param   segments                   number of line segments
    VecDataPoints _makeCubeBezier(const DataPoint& p0, const DataPoint& p1, const DataPoint& p2, const DataPoint& p3, int segments = 100) {
        VecDataPoints points;
        for (int i = 0; i <= segments; ++i) {
            number t = static_cast<number>(i) / segments;
            number u = 1.0f - t;
            number x = u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x;
            number y = u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y;
            DataPoint p = {x, y};
            points.push_back(p);
        }
        return points;
    };
    
    void _scaleDataSet(DataSet &ds, Point2D scale) {
        ds.setScale(scale.x, scale.y);
    }
    
    void _rotateDataSet(DataSet &ds, number angle, Point2D anchor) {
        ds.setRotaion(angle, anchor);
    }
    
public:
    
    ildacompose(const atoms& args = {}) {
        if(!dummy()) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            srand((unsigned int)ts.tv_nsec);
            uint rand_id = rand();
            std::string id_string = std::to_string(rand_id);
            std::ostringstream ss;
            ss << std::setw(12) << std::setfill('0') << id_string;
            this->_instance_id = "ild" + std::string(ss.str());
                // get the pointer to jam.ilda.manager max-object
            this->_manager = (c74::max::t_object*)c74::max::object_new_typed(c74::max::CLASS_NOBOX, symbol("jam.ilda.manager"), 0, NULL);
                // get the pointer to jam.ilda.manager max-object's struct
            this->_manager_struct_ptr = (t_jam_im *)typedmess(this->_manager,symbol("get_struct"),0,0L);
            this->_updateFonts();
            this->font("Arial");
            this->_appendEmptyFrame();
        }
    };
    
    ~ildacompose() {};
    
    MIN_DESCRIPTION     { "Create and modify ILDA files for laser animation. <br /><o>jam.ilda.compose</o> can create ilda files for laser animation by drawing or writing into frames, save them to disk and make them accessible to other jam.ilda.* objects" };
    MIN_TAGS            { "ILDA" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file, jam.jit.gl.ilda.sketch, jam.ilda.dict, jam.helios"};
//    MIN_FLAGS           {behavior_flags::nobox};
    
    inlet<> input_1             { this, "ILDA file reference", "anything" };
    outlet<> o_file_reference   { this, "ilda file reference"  };
    outlet<> o_font_faces       { this, "Pobulate a umenu with availeble fonts"  };
    outlet<> o_edit_frame       { this, "Frame currently selected for editing", "int"};
    outlet<> o_framecount       { this, "Number of frames created", "int"};
    outlet<> o_file_result      { this, "file opration success/failure notification", "list" };
    
    
    attribute<symbol> companyname {
        this, "companyname", "NOT_SET",
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<std::string>(args, &cleaned_args, 1, "NOT_SET");
                std::string name = static_cast<std::string>(cleaned_args[0]);
                if(name == "") {
                    name = "NOT_SET";
                }
                this->_formatIldaString(name);
                this->_company_name = name;
                this->_updateFrameHeaders();
                if(this->initialized()) {
                    this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
                    this->_updateOutlets();
                }
                cleaned_args[0] = name;
                return cleaned_args;
            }
        },
        title {"Company Name"},
        description {"Set the <i>Company Name</i> in the headers of the ILDA file.<br />ILDA files contain of a sequence of <i>frames</i>. Every <i>frame</i> has a <i>header</i> summarizing some information. This attribute sets the value of the header field 'Company Name' (max 8 ASCII characters). "},
        category {"ILDA File"}
    };
    
    attribute<symbol> frameprefix {
        this, "frameprefix", "",
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<std::string>(args, &cleaned_args, 1, "");
                std::string name = static_cast<std::string>(cleaned_args[0]);
                this->_formatIldaString(name, 5);
                this->_frame_name_prefix = name;
                this->_updateFrameHeaders();
                this->_updateFrameHeaders();
                if(this->initialized()) {
                    this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
                    this->_updateOutlets();
                }
                cleaned_args[0] = name;
                return cleaned_args;
            }
        },
        title {"Frame Name Prefix"},
        description {"Set a <i>Frame Name</i> prefix in the headers of the ILDA file.<br />ILDA files contain of a sequence of <i>frames</i>. Every <i>frame</i> has a <i>header</i> summarizing some information. By default <o>jam.ilda.compose</o> uses the frame index as name. This attribute sets an optional prefix to the <i>Frame Name</i> (max 4 ASCII characters)."},
        category {"ILDA File"}
    };
    
    attribute<bool> kerning {
        this, "kerning", true,
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<bool>(args, &cleaned_args, 1, true);
                this->_kerning = cleaned_args[0];
                return cleaned_args;
            }
        },
        title {"Kerning"},
        description{"Apply kerning to text rendering.<br/>Kerning is the adjustment of space between specific pairs of characters in a font to improve visual appearance and readability."},
        category{"Text Rendering"},
    };
    
    attribute<number> lineheight {
        this, "lineheight", 1.,
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 1, 1.);
                number lineheight = cleaned_args[0];
                cleaned_args[0] = std::clamp(lineheight, 0.1, 5.);
                return cleaned_args;
            }
        },
        title { "Line Height" },
        description { "Set the hine height.<br/>This applies when using the message <m>pen up</m> or <m>pen down</m>" },
        category{"Text Rendering"},
    };
    
    attribute<symbol> textalign {
        this, "textalign", "center",
        range {"left", "center", "right"},
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<std::string>(args, &cleaned_args, 1, "center");
                std::string value = cleaned_args[0];
                if(value != "left" && value != "right" && value != "center") {
                    cleaned_args[0] = "center";
                }
                return cleaned_args;
            }
        },
        title { "Text Align" },
        description { "Set text alignment mode relative to the current pen position (default = center)<br/>Possible values: <br/><ul><li>left</li><li>center</li><li>right</li></ul>" },
        category{"Text Rendering"},
    };
    
    attribute<number> textfontsize {
        this, "textfontsize", 36,
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<number>(args, &cleaned_args, 1, 20.);
                cleaned_args[0] = std::clamp(static_cast<number>(cleaned_args[0]), 10., 1000.);
                this->_font_size = (number)cleaned_args[0] / 20.;
                return cleaned_args;
            }
        },
        title {"Font Size"},
        description{"Set the font size for text rendering."},
        category{"Text Rendering"},
        visibility{visibility::show}
    };

    
    message<>bang  {
        this, "bang", "Output the ILDA file reference out of the leftmost outlet.",
        MIN_FUNCTION {
            this->_updateOutlets();
            return {};
        }
    };
    
    message<threadsafe::no> clear {
        this, "clear", "Remove all frames.",
        MIN_FUNCTION {
            // clear raw frames
            this->_data_sets.clear();
            this->_ilda_frames.clear();
            this->_getStructPointer()->clearInstanceFile(this->_instance_id);
            this->_appendEmptyFrame();
            return {};
        }
    };
    
    message<threadsafe::no> seteditframe {
        this, "seteditframe", "Select frame to be edited. Starts at 0 (zero)",
        MIN_FUNCTION {
            if(args.size() == 0) {
                cwarn << "missing argument for message 'seteditframe'" << endl;
                return {};
            }
            if (args.size() > 1) {
                cwarn << "extra argument for message 'seteditframe'" << endl;
            }
            
            if(
               args[0].type() != message_type::int_argument
               && args[0].type() != message_type::float_argument) {
                   cwarn << args[0] << " bad number" << endl;
                   return {};
               }
            
            int frame_index = args[0];
            
            if(frame_index < 0 || frame_index > this->_ilda_frames.size() - 1) {
                return {};
            }
            
            this->_edit_frame = frame_index;
            
            atoms msg_atoms;
            queued_message_t msg;
            
            msg_atoms.clear();
            msg_atoms.push_back(this->_edit_frame);
            msg.set(&o_edit_frame, msg_atoms);
            msg.send(this);
            
            
            return {};
        }
        
    };
    
    message<threadsafe::no> geteditframe {
        this, "geteditframe", "Outputs the currenlty selected frame for editing out of the third outlet.",
        MIN_FUNCTION {
            atoms msg_atoms;
            queued_message_t msg;
            
            msg_atoms.clear();
            msg_atoms.push_back(this->_edit_frame);
            msg.set(&o_edit_frame, msg_atoms);
            msg.send(this);
            return {};
        }
    };
    
    message<threadsafe::no> appendframe {
        this, "appendframe", "Append a new (empty) frame.",
        MIN_FUNCTION {
            this->_appendEmptyFrame();
            return {};
        }
    };
    
    message<threadsafe::no> removeframe {
        this, "removeframe", "Remove frame a frame. If no argument is provided the currently set edit frame is removed. If one argument [frame index] is present, the frame at [frame index] will be removed.",
        MIN_FUNCTION {
            if(this->_ilda_frames.size() == 0) {
                return {};
            }
            int frame_index = -1;
            if(args.size() == 0) {
                frame_index = (int)this->_edit_frame;
            } else {
                if(args[0].type() != message_type::int_argument && args[0].type() != message_type::float_argument) {
                       cwarn << args[0] << " bad number" << endl;
                       return {};
                   }
                frame_index = args[0];
            }
            
            if(frame_index < 0 || frame_index > this->_ilda_frames.size() -1) {
                cwarn << "frame index out of range" << endl;
                return {};
            }

            this->_ilda_frames.erase(this->_ilda_frames.begin() + frame_index);
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            
            if(this->_edit_frame > this->_ilda_frames.size() - 1) {
                this->_edit_frame = (this->_ilda_frames.size() > 0) ? this->_ilda_frames.size() - 1 : 0;
            }
            
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no> duplicateframe {
        this, "duplicateframe", "Duplicate a frame. If no argument is provided the currently set edit frame is duplicated. If one argument [frame index] is present, the frame at index will be duplicated.",
        MIN_FUNCTION {
            if(this->_ilda_frames.size() == 0) {
                return {};
            }
            int frame_index = -1;
            if(args.size() < 1) {
                frame_index = (int)this->_edit_frame;
            } else {
                if(args[0].type() != message_type::int_argument && args[0].type() != message_type::float_argument) {
                       cwarn << args[0] << " bad number" << endl;
                       return {};
                   }
                frame_index = args[0];
            }
            if(frame_index < 0 || frame_index > this->_ilda_frames.size() - 1) {
                cwarn << "frame index out of range" << endl;
                return {};
            }
            
            DataSet d = this->_data_sets[frame_index];
            jam::ilda::IldaFrame f = d.toIldaFrame();
            this->_data_sets.insert(this->_data_sets.begin() + frame_index, d);
            this->_ilda_frames.insert(this->_ilda_frames.begin() + frame_index, f);
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
        
    };
    
    message<threadsafe::no> copyframe {
        this, "copyframe", "Copy a frame. The message <m>copyframe</m> followed by two arguments <i>source_index</i> <i>destination_index</i> copies a frame from <i>source_index</i> to <i>destination_index</i>.",
        MIN_FUNCTION {
            if(this->_ilda_frames.size() == 0) {
                return {};
            }
            if(args.size() < 2 ) {
                return {};
            }
            if(args[0].type() != message_type::int_argument && args[0].type() != message_type::float_argument) {
                cwarn << args[0] << " bad number" << endl;
                return {};
            }
            if(args[1].type() != message_type::int_argument && args[1].type() != message_type::float_argument) {
                cwarn << args[1] << " bad number" << endl;
                return {};
            }
            
            
            int source_index = args[0];
            int dest_index = args[1];
            
            if(source_index < 0 || source_index > this->_ilda_frames.size() - 1 ) {
                cwarn << "source_index out of range" << endl;
                return {};
            }
            
            if(dest_index < 0) {
                cwarn << "destination_index out of range" << endl;
                return {};
            }
    
            DataSet d = this->_data_sets[source_index];
            
            jam::ilda::IldaFrame f = d.toIldaFrame();
            if(dest_index >= this->_ilda_frames.size()) {
                this->_data_sets.push_back(d);
                this->_ilda_frames.push_back(f);
            } else {
                this->_data_sets.insert(this->_data_sets.begin() + dest_index, d);
                this->_ilda_frames.insert(this->_ilda_frames.begin() + dest_index, f);
            }
            
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            return {};
        }
    };
    
    message<threadsafe::no> moveto {
        this, "moveto", "Move to point x/y without drawing a line. (laser is blanked)", {
            MIN_FUNCTION {
                if(args.size() < 2) {
                    cwarn << "missing argument for message 'moveto'" << endl;
                    return {};
                }
                if (this->_ilda_frames.size() == 0) {
                    this->_appendEmptyFrame();
                }
                number x = args[0];
                number y = args[1];
                
                DataPoint dp;
                dp.x = x;
                dp.y = y;
                dp.blanking = true;
                
                this->_data_sets[this->_edit_frame].addDataPoint(dp);
                this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
                this->_updateFrameHeaders();
                this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
                this->_updateOutlets();
                
                return {};
            }
        }
    };
    
    message<threadsafe::no> svg {
        this, "svg", "Parse a SVG file into the current edit frame. <br/><br/><b>Note:</b> This feature is experimental at best. SVG files are more complex than simple line-segment graphics that use ILDA files. The result may vary a lot depending on the source file.",
        MIN_FUNCTION {
            if(this->_getSvgParsingState()) {
                cwarn << "file loading already in progress" << endl;
                return {};
            }
            
            this->_setSvgParsingState(true);
            
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
                open_result = c74::max::open_dialog(_filename, &path, &outtype, &filetype, (short)1);
                if(open_result != c74::max::MAX_ERR_NONE) {
                    if(open_result < c74::max::MAX_ERR_NONE) {
                        cerr << "couldn't open file" << endl;
                        msg_atoms.clear();
                        msg_atoms.push_back("svg");
                        msg_atoms.push_back(_filename);
                        msg_atoms.push_back(0);
                        msg.set(&o_file_result, msg_atoms);
                        msg.send(this);
                    }
                    this->_setSvgParsingState(false);
                    return{};
                }
            } else {
                std::string user_filename = this->_import_args[0];
                if(user_filename.size() > c74::max::MAX_PATH_CHARS - 1) {
                    cerr << "file name too long" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("svg");
                    msg_atoms.push_back(_filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                    this->_setSvgParsingState(false);
                    return {};
                    
                }
                strcpy(_filename, user_filename.c_str());
                
                open_result = c74::max::locatefile_extended(_filename, &path, &outtype, &filetype, (short)1);
                if(open_result != c74::max::MAX_ERR_NONE) {
                    cerr << "couldn't open file" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("svg");
                    msg_atoms.push_back(_filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                    this->_setSvgParsingState(false);
                    return {};
                }
            }
            
            open_result = c74::max::path_opensysfile( _filename, path, &_file_handle,c74::max::READ_PERM);
            
            if(open_result != c74::max::MAX_ERR_NONE) {
                cerr << "couldn't open file" << endl;
                msg_atoms.clear();
                msg_atoms.push_back("svg");
                msg_atoms.push_back(_filename);
                msg_atoms.push_back(0);
                msg.set(&o_file_result, msg_atoms);
                msg.send(this);
                this->_setSvgParsingState(false);
                return {};
            }
            
            unsigned long size;
            c74::max::t_max_err read_result = 0;
            c74::max::t_handle file_content_handle = nullptr;
            std::string file_content = "";
            c74::max::sysfile_geteof(_file_handle,&size);
            
            size = c74::max::sysmem_handlesize(file_content_handle);
            
            if (!(file_content_handle = c74::max::sysmem_newhandle(size))) {
                cerr << "not enough memory to open " << _filename << endl;
                msg_atoms.clear();
                msg_atoms.push_back("svg");
                msg_atoms.push_back(_filename);
                msg_atoms.push_back(0);
                msg.set(&o_file_result, msg_atoms);
                msg.send(this);
                this->_setSvgParsingState(false);
                return {};
            }
            
                // https://cycling74.com/forums/t_handle-and-sysmem_newhandle-crash-help-needed
            read_result = c74::max::sysfile_readtextfile(_file_handle,file_content_handle,size, c74::max::TEXT_ENCODING_USE_FILE);
            if(read_result != c74::max::MAX_ERR_NONE) {
                c74::max::sysmem_freehandle(file_content_handle);
                cerr << "couldn't read file" << endl;
                msg_atoms.clear();
                msg_atoms.push_back("svg");
                msg_atoms.push_back(_filename);
                msg_atoms.push_back(0);
                msg.set(&o_file_result, msg_atoms);
                msg.send(this);
                this->_setSvgParsingState(false);
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
                msg_atoms.push_back("svg");
                msg_atoms.push_back(_filename);
                msg_atoms.push_back(0);
                msg.set(&o_file_result, msg_atoms);
                msg.send(this);
                this->_setSvgParsingState(false);
                return {};
                
            }
            
            msg_atoms.clear();
            msg_atoms.push_back("svg");
            msg_atoms.push_back(_filename);
            msg_atoms.push_back(1);
            msg.set(&o_file_result, msg_atoms);
            msg.send(this);
            this->_setSvgParsingState(false);
            
            
                // parse SVG data into shapes
            _svg_shapes.clear();
            for (NSVGshape* shape = image->shapes; shape != nullptr; shape = shape->next) {
                jam::Shape s("SVGPath");
                
                if(shape->stroke.type == NSVG_PAINT_COLOR) {
                        // AAAAAAAA BBBBBBB  GGGGGGGG RRRRRRRR
                        // 11111111 11111111 11111111 11111111
                    unsigned int strokeColor = shape->stroke.color;
                    strokeColor = strokeColor & 0x00FFFFFF; // Eliminate Alpha
                    unsigned int r = (strokeColor &  0x000000FF);
                    unsigned int g = (strokeColor &  0x0000FF00) >> 8;
                    unsigned int b = (strokeColor &  0x00FF0000) >> 16;
                        // Black doesn't exist in lasers, so we change it to white
                    if(r + g + g == 0) {
                        r = g = b = 255;
                    }
                    jam::RGBColor color;
                    color.r = static_cast<uint8_t>(r);
                    color.g = static_cast<uint8_t>(g);
                    color.b = static_cast<uint8_t>(b);
                    s.setColor(color);
                }
                
                for (NSVGpath* path = shape->paths; path != nullptr; path = path->next) {
                    for (int i = 0; i < path->npts; ++i) {
                        number x = path->pts[i * 2];       // x coordinate
                        number y = path->pts[i * 2 + 1];   // y coordinate
                                                          // Y-Axis Flip
                        y = image->height - y;
                            // Normalize to [-1, 1]
                        number nx = (x / image->width) * 2.0f - 1.0f;
                        number ny = (y / image->height) * 2.0f - 1.0f;
                        s.addPoint({nx, ny});
                    }
                    
                    if (!s.getPoints().empty()) {
                            // Close path if marked closed
                        if (path->closed) {
                            s.addPoint(s.getPoints().front());
                        }
                        s.thinShape();
                    }
                    this->_svg_shapes.push_back(s);
                }
                
            }
            
            nsvgDelete(image);
            
                // Add shape data to current edit_frame
            if(this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            
            DataSet ds = this->_data_sets[this->_edit_frame];
            for(size_t i = 0; i< this->_svg_shapes.size(); i++) {
                for(size_t j = 0; j < this->_svg_shapes[i].getPoints().size(); j++) {
                    bool is_blanking = (j == 0);
                    Point2D p = this->_svg_shapes[i].getPoints()[j];
                    DataPoint dp;
                    dp.r = is_blanking ? 0 : static_cast<number>(this->_svg_shapes[i].getColor().r) * 255.;
                    dp.g = is_blanking ? 0 : static_cast<number>(this->_svg_shapes[i].getColor().g) * 255.;
                    dp.b = is_blanking ? 0 : static_cast<number>(this->_svg_shapes[i].getColor().b) * 255.;
                    dp.blanking = is_blanking;
                    dp.x = p.x;
                    dp.y = p.y;
                    ds.addDataPoint(dp);
                }
                // close shape
                Point2D p = this->_svg_shapes[i].getPoints()[0];
                DataPoint dp;
                dp.r = static_cast<number>(this->_svg_shapes[i].getColor().r) * 255.;
                dp.g =  static_cast<number>(this->_svg_shapes[i].getColor().g) * 255.;
                dp.b = static_cast<number>(this->_svg_shapes[i].getColor().b) * 255.;
                dp.blanking = false;
                dp.x = p.x;
                dp.y = p.y;
                ds.addDataPoint(dp);
                
            }
            this->_data_sets[this->_edit_frame] = ds;
            this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            
            this->_updateOutlets();
            this->_setSvgParsingState(false);
            
            return {};
        }
    };
    
    message<threadsafe::no> getfonts {
        this, "getfonts", "Get a list of fonts.<br/>Populate a <o>umenu</o> connected to the second outlet with available fonts.<br/><b>Note:</b> only system wide installed True Type Fonts (ttf) are supported.",
        MIN_FUNCTION {
            atoms msg_atoms;
            queued_message_t msg;
            
            msg_atoms.clear();
            msg_atoms.push_back("clear");
            msg.set(&o_font_faces, msg_atoms);
            msg.send(this);
            
            for (const auto& [name, path] : this->_available_fonts) {
                msg_atoms.clear();
                msg_atoms.push_back("append");
                msg_atoms.push_back(name);
                msg.set(&o_font_faces, msg_atoms);
                msg.send(this);
            }
            
            return {};
        }
    };
    
    message<threadsafe::no> font {
        this, "font", "Loads a TTF font face.<br/> The message <m>font</m> followed by a font name will load the font for writing text into a frame using the <m>text</m> message.",
        MIN_FUNCTION {
            if(args.size() > 0) {
                atoms msg_atoms;
                queued_message_t msg;

                std::string font_name = args[0];
                if (this->_available_fonts.find(font_name) == this->_available_fonts.end()) {
                    cwarn << "font '"<< font_name << "' not found or not a TTF font" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("font");
                    msg_atoms.push_back(font_name);
                    msg_atoms.push_back(0);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                    return {};
                } else {
                    std::string font_path = this->_available_fonts[font_name].file_path;
                    short path = 0;
                    short open_result;
                    c74::max::t_fourcc outtype;
                    font_path.resize(c74::max::MAX_PATH_CHARS);
                    char c_font_path[c74::max::MAX_PATH_CHARS] = {0};
                    strcpy(c_font_path, font_path.c_str());
                    open_result = c74::max::locatefile_extended(c_font_path, &path, &outtype, NULL, 0);
                    if(open_result != c74::max::MAX_ERR_NONE) {
                        cerr << "Couldn't open file" << endl;
                        msg_atoms.clear();
                        msg_atoms.push_back("font");
                        msg_atoms.push_back(font_name);
                        msg_atoms.push_back(0);
                        msg.set(&o_file_result, msg_atoms);
                        msg.send(this);
                        return {};
                    }
                    c74::max::t_filehandle file_handle;
                    open_result = c74::max::path_opensysfile( c_font_path, path, &file_handle,c74::max::READ_PERM);
                    if(open_result != c74::max::MAX_ERR_NONE) {
                        cerr << "Couldn't open file" << endl;
                    
                        msg_atoms.clear();
                        msg_atoms.push_back("font");
                        msg_atoms.push_back(font_name);
                        msg_atoms.push_back(0);
                        msg.set(&o_file_result, msg_atoms);
                        msg.send(this);
                        return {};
                    }
                    
                    c74::max::t_max_err read_result = 0;
                    c74::max::t_ptr_size chunk_size = BINARY_FILE_CHUNK;
                    char file_buffer[BINARY_FILE_CHUNK];
                    std::vector<unsigned char>font_buffer;
                    while(true) {
                        read_result = c74::max::sysfile_read(file_handle,&chunk_size,file_buffer);
                        for(size_t i = 0; i < chunk_size; i++) {
                            font_buffer.push_back(file_buffer[i]);
                        }
                        if (read_result < 0) {
                            break;
                        }
                    }
                    
                    this->_ttfFileProcessor.setFileData(font_buffer);
                    int success = 1;
                    if(this->_ttfFileProcessor.initFont(this->_available_fonts[font_name].face_index) != jam::ttf::FontError::NO_ERROR) {
                        success = 0;
                    };
                    
                    
                    msg_atoms.clear();
                    msg_atoms.push_back("font");
                    msg_atoms.push_back(font_name);
                    msg_atoms.push_back(success);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                }
            }
            return {};
        }
        
    };
    message<>pen {
        this, "pen", "Set the pen position for wryting text. <br/>The message <m>pen</m> followed by 2 floats will set the writing position.<br/>The message <m>pen up</m> or <m>pen down</m> will move the writing position one line up or down respectively.",
        MIN_FUNCTION {
            if(args.size() > 0) {
                if(args[0].type() == message_type::symbol_argument) {
                    if(args[0] == "up") {
                        this->_pen_pos.y += std::clamp(this->_ttfFileProcessor.getLineHeight(this->_font_size), -1., 1.) * this->lineheight;
                    }
                    if(args[0] == "down") {
                        this->_pen_pos.y -= std::clamp(this->_ttfFileProcessor.getLineHeight(this->_font_size), -1., 1.) * this->lineheight;
                    }
                    
                }
            }
            if(args.size() >= 2
               && (args[0].type() == message_type::float_argument || args[0].type() == message_type::int_argument)
               && (args[1].type() == message_type::float_argument || args[1].type() == message_type::int_argument)
               ) {
                this->_pen_pos.x = std::clamp(static_cast<number>(args[0]), -1., 1.);
                this->_pen_pos.y = std::clamp(static_cast<number>(args[1]), -1., 1.);
            }
            return {};
        }
    };
    
    message<threadsafe::no> text {
        this, "text", "Write a text to the current edit frame.",
        MIN_FUNCTION {
            std::string in_string = "";
            if(args.size() < 1) {
                return {};
            }
            for(size_t i = 0; i < args.size(); i++) {
                if(i > 0) {
                    in_string += " ";
                }
                in_string += static_cast<std::string>(args[i]);
            }
        
            VecGlyphPoints glyph_points = this->_ttfFileProcessor.getGlyphVertices(
                 in_string,
                 this->_pen_pos,
                 this->_kerning,
                 this->_font_size,
                 this->textalign
            );
            if (this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            for(size_t i = 0; i < glyph_points.size(); i++) {
                bool blanking = glyph_points[i].type == jam::ttf::VertexType::MoveTo;
                DataPoint dp;
                dp.r = this->_color.r * !blanking;
                dp.g = this->_color.g * !blanking;
                dp.b = this->_color.b * !blanking;
                dp.blanking = blanking;
                dp.x = glyph_points[i].pos.x;
                dp.y = glyph_points[i].pos.y;
                this->_data_sets[this->_edit_frame].addDataPoint(dp);
            }

            this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no>line {
        this, "line", "Draw a line into a frame.",
        MIN_FUNCTION {
            if(args.size() < 4) {
                cwarn << "missing argument for message 'line'" << endl;
                return {};
            }
            if (this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            number x_start = args[0];
            number y_start = args[1];
            number x_end   = args[2];
            number y_end   = args[3];
            
            
            // create raw data points
            DataPoint raw_start;
            raw_start.x = x_start;
            raw_start.y = y_start;
    
            
            DataPoint raw_end;
            raw_end.x = x_end;
            raw_end.y = y_end;
            VecDataPoints points;
            points.push_back(raw_start);
            points.push_back(raw_end);
        
            this->_addDataPointsToEditDataSet(points);
            this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();

            return {};
        }
    };
    
    message<threadsafe::no>circle {
        this, "circle", "Draw a circle into a frame",
        MIN_FUNCTION {
            if(args.size() < 3) {
                cwarn << "missing argument for message 'circle'" << endl;
                return {};
            }
            if (this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            
                // center point
            DataPoint c = {(number)args[0], (number) args[1]};
            
                // radius x/y
            DataPoint radius = {(number)args[2], (number)args[2]};
            
            number t_start = 0;
            number t_end = 360;
            if(args.size() >= 5) {
                t_start = (number)args[3];
                t_end = (number)args[4];
            }
            
            int seg = 50;
            if(args.size() >= 6) {
                seg = (int)args[5];
                seg = (seg < 3) ? 3 : seg;
                seg = (seg > 200) ? 200 : seg;
            }
            VecDataPoints points = this->_makeEllipse(c, radius, t_start, t_end, seg);
            this->_addDataPointsToEditDataSet(points);
            this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
            
        }
    };
    
    message<threadsafe::no>ellipse {
        this, "ellipse", "Draw an ellipse into a frame",
        MIN_FUNCTION {
                // ellipse x_center y_center x_radius y_radius r g b segments
            if(args.size() < 4) {
                cwarn << "missing argument for message 'circle'" << endl;
                return {};
            }
            if (this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            
                // center point
            DataPoint c = {(number)args[0], (number) args[1]};
            
                // radius x/y
            DataPoint radius = {(number)args[2], (number)args[3]};
            
            
            number t_start = 0;
            number t_end   = 360;
            if(args.size() >= 6) {
                t_start = (number)args[4];
                t_end = (number)args[5];
            }
            
            int seg = 50;
            if(args.size() >= 7) {
                seg = (int)args[6];
                seg = (seg < 3) ? 3 : seg;
                seg = (seg > 200) ? 200 : seg;
            }
            
            VecDataPoints points = this->_makeEllipse(c, radius, t_start, t_end, seg);
            this->_addDataPointsToEditDataSet(points);
            this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no>rect {
        this, "rect", "Draw a rectangle into a frame.",
        MIN_FUNCTION {
                // rect x_topleft y_topleft x_bottomright y_bottomright r g b corner-radius segment
            if(args.size() < 4) {
                cwarn << "missing argument for message 'rect'" << endl;
                return {};
            }
            if (this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            
            number tl_x = args[0];
            number tl_y = args[1];
            number br_x = args[2];
            number br_y = args[3];
            
            number border_radius = 0.;
            if(args.size() >= 5) {
                border_radius = std::clamp((number)args[4], 0., 1.);
            }
            int seg = 10;
            if(args.size() >= 6) {
                seg = (int)args[5];
                seg = (seg < 1) ? 1 : seg;
                seg = (seg > 200) ? 200 : seg;
            }
            DataPoint tl = {tl_x, tl_y};
            DataPoint br = {br_x, br_y};
            VecDataPoints points = this->_makeRectangle(tl, br, border_radius);
            
            this->_addDataPointsToEditDataSet(points);
            this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no>bezier {
        this, "bezier", "Draw a cubic bezier curve into a frame",
        MIN_FUNCTION {
                //besier x_start y_start x_c1 y_c1 x_c2 y_c2 x_end y_end
            if(args.size() < 8) {
                cwarn << "missing argument for message 'bezier'" << endl;
                return {};
            }
            if (this->_ilda_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
                // start point
            DataPoint start = {(number)args[0], (number) args[1]};
            
                // control point 1
            DataPoint c1 = {(number)args[2], (number)args[3]};
            
                // control point 2
            DataPoint c2 = {(number)args[4], (number)args[5]};
            
                // end point
            DataPoint end = {(number)args[6], (number)args[7]};
            
            
            int seg = 50;
            if(args.size() >= 9) {
                seg = (int)args[8];
                seg = (seg < 3) ? 3 : seg;
                seg = (seg > 200) ? 200 : seg;
            }
            
            VecDataPoints points = this->_makeCubeBezier(start, c1, c2, end, seg);
            this->_addDataPointsToEditDataSet(points);
            this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no>ilda {
        this, "ilda", "Reference to am ILDA file loaded by <o>jam.ilda.file</o>. The frames from the file will be appended.<br/><b>Note:</b>3D frames will be flattened to 2D frames by discarting the y axis. Frames using indexed colors, they are converted to true color mode",
        MIN_FUNCTION {
            if(args.size() < 1) {
                cwarn << "missing argument for message 'ilda'" << endl;
                return {};
            }
            std::string ilda_file_refence = args[0];
            std::vector<jam::ilda::IldaFrame> frames = this->_getStructPointer()->getFrames(ilda_file_refence);
            this->_parseFrames2DToTrueColor(frames);
            
            VecDataSets data_sets = DataSet::framesToDataSets(frames);
            
            for(auto it = data_sets.begin(); it < data_sets.end(); it++) {
                this->_data_sets.push_back(*it);
                this->_ilda_frames.push_back(it->toIldaFrame());
            }
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            return {};
        }
    };
    
    message<threadsafe::no> reverseframes {
        this, "reverseframes", "Reverse the order of the frames.",
        MIN_FUNCTION {
            std::reverse(this->_data_sets.begin(), this->_data_sets.end());
            std::reverse(this->_ilda_frames.begin(), this->_ilda_frames.end());
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            return {};
            
        }
    };
    
    message<threadsafe::no>rotateframe {
        this, "rotateframe", "Rotate a frame. If one argument follows the message <m>rotateframe</m>, the frame will be rotated around the center point. If three argument follow the message <m>rotateframe</m>, the sencond and third arguments specify the rotation anker.",
        MIN_FUNCTION {
            if(args.size() < 1) {
                cwarn << "missing argument for message 'rotateframe'" << endl;
                return {};
            }
            if (this->_ilda_frames.size() == 0) {
                return {};
            }
            number angle = args[0];
            Point2D anchor = {0., 0.};
            if(args.size() >= 3) {
                anchor.x = (number)args[1];
                anchor.y = (number)args[2];
            }
            this->_rotateDataSet(this->_data_sets[this->_edit_frame], angle, anchor);
            this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no>scaleframe {
        this, "scaleframe", "Scale a frame. <br/>If one argument follows the message <m>scaleframe</m> x and y axis are scaled by that value.<br />If two arguments follow the message <m>scaleframe</m>, the first argument specifies the scaling along the x-axis, the second along the y-axis.",
        MIN_FUNCTION {
            if(args.size() < 1) {
                cwarn << "missing argument for message 'scaleframe'" << endl;
                return {};
            }
            if (this->_ilda_frames.size() == 0) {
                return {};
            }
            number scale_x = args[0];
            number scale_y = scale_x;
            
            if(args.size() >= 2) {
                scale_y = args[1];
            }
            
            Point2D scale_factors = {scale_x, scale_y};
            this->_scaleDataSet(this->_data_sets[this->_edit_frame], scale_factors);
            this->_ilda_frames[this->_edit_frame] = this->_data_sets[this->_edit_frame].toIldaFrame();
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<>drawcolor {
      this, "drawcolor", "Set the drawing color.",
        MIN_FUNCTION {
            if(args.size() < 3) {
                cwarn << "missing argumnet for message 'drawcolor'. Expected 3 floats" << endl;
                return {};
            }
    
            // RAW set raw color
            this->_color.r = std::clamp(static_cast<number>(args[0]), 0., 1.);
            this->_color.g = std::clamp(static_cast<number>(args[1]), 0., 1.);
            this->_color.b = std::clamp(static_cast<number>(args[2]), 0., 1.);
            
            return {};
        }
    };
    
    message<>export_file {
        this, "export", "Write the frames to ILDA file. If no path/filename is provided, a dialog will be presented. A success/failure notification will be sent to the rightmost outlet in the form <m>export filename 1/0.</m>",
        MIN_FUNCTION {
            atoms msg_atoms;
            queued_message_t msg;
            
            jam::ilda::ParseResult result = jam::ilda::ParseResult::ERROR;
            
            char                      filename[c74::max::MAX_PATH_CHARS] = {0};
            short                     path = 0;
            c74::max::t_fourcc        types[1] = {'ILDA'};
            c74::max::t_fourcc        outtype = 0;
            c74::max::t_max_err       err;
            c74::max:: t_filehandle   fh;
            
            if(args.size() > 0) {
                // some basic sanity checks
                std::string input_filename = args[0];
                // check if it ends with .ild
                std::string suffix = ".ild";
                bool has_suffix;
                if(input_filename.length() < suffix.length()) {
                    has_suffix = false;
                } else {
                    has_suffix = (0 == input_filename.compare(input_filename.length() - suffix.length(), suffix.length(), suffix));
                }
                 
                if(!has_suffix) {
                    input_filename += suffix;
                }
                if(input_filename.size() > c74::max::MAX_PATH_CHARS) {
                    cwarn << "invalid filename" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("export");
                    msg_atoms.push_back(input_filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                    return {};
                }
                
                strcpy(filename, input_filename.c_str());
            } else {
                c74::max::saveas_promptset("Export as file...");
                err = c74::max::saveasdialog_extended(filename, &path, &outtype, types, 1);
                if (err) {       // User Cancelled
                    return {};
                }
            }
            
            // remove empry frames to be sure to create a valid ILDA file
            this->_removeEmptyFrames();
            
            
                // First: Create File
            err = c74::max::path_createsysfile(filename, path, 'ILDA', &fh);
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_ilda_frames, std::string(""));
            this->_updateOutlets();
            
            if(err == c74::max::MAX_ERR_NONE) {
                result = jam::ilda::ParseResult::SUCCESS;
                std::vector<unsigned char> file_bytes;
                result = this->_ildaFileProcessor.parseFramesToFileData(file_bytes, this->_ilda_frames);
                unsigned long byte_count = file_bytes.size();
                    // Second: Write File
                unsigned char *raw_data = reinterpret_cast<unsigned char *>(malloc(file_bytes.size() * sizeof(unsigned char)));
                
                for(unsigned long i = 0; i < byte_count; i++) {
                    raw_data[i] = file_bytes[i];
                }
                
                err = c74::max::sysfile_write(fh, &byte_count ,raw_data);
                    // Third: Close File
                c74::max::sysfile_seteof(fh, byte_count);
                c74::max::sysfile_close(fh);
                free(raw_data);
                if(err != c74::max::MAX_ERR_NONE) {
                    result = jam::ilda::ParseResult::ERROR;
                }
            } else {
                result = jam::ilda::ParseResult::ERROR;
            }
            
            msg_atoms.clear();
            msg_atoms.push_back("export");
            msg_atoms.push_back(filename);
            msg_atoms.push_back(result == jam::ilda::ParseResult::SUCCESS);
            msg.set(&o_file_result, msg_atoms);
            msg.send(this);
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


MIN_EXTERNAL(ildacompose);


