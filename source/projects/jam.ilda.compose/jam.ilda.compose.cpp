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
using Point2D = jam::Point2D;
using VecPoint2D = std::vector<jam::Point2D>;
using VecGlyphPoints = std::vector<jam::ttf::GlyphVertex>;



class ildacompose : public object<ildacompose>
{
private:

    std::string _instance_id = "";                  // Unique ID for each object instance.
                                                    // Used to itentify loaded ILDA filed data in the global jam.ilda.manager
    c74::max::t_object *_manager;                   // Pointer to global jam.ilda.manager object
                                                    // (stores data to be accasibele by other jam.ilda.* object)
    t_jam_im * _manager_struct_ptr = NULL;          // Pointer to max-object struct of the jam.ilda.manager object
    
    atoms _import_args;                             // Stores arguments of import message,
                                                    // to be accasible in the scope of the file loader thread
    c74::max::t_filehandle file_handle;             // File handle for importing SVG files.
    
    char filename[c74::max::MAX_PATH_CHARS] = {0};  // File name of ILDA file toi be imported
    
    std::vector<jam::Shape> _shapes;           // Vector of Shapes from parsed SVG file
    
    std::string _company_name = "NOT_SET";          // Company name set to frame headers
    
    std::string _frame_name_prefix = "";            // Prefix for frame name set to frame headers
    
    size_t _edit_frame = 0;
    
    jam::ilda::IldaFileProcessor _ildaFileProcessor;     // Class with functions for ILDA file processing/parsing
    
    jam::ttf::TtfFileProcessor _ttfFileProcessor;
    
    protected :
    

    std::map<std::string, std::string>_available_fonts;
    

    
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
        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;
    } rgb_color_t;
    
    rgb_color_t _color;
    
    jam::ttf::Point2D _pen_pos = {-1., 1.};
        /// FIFO queue for messages to be sent to outlets
    fifo<queued_message_t> _to_max_queue { 1000 };
    
        /// Mutex lock for outlet message thread safty
    std::mutex _enqueue_msg_lock;
    
    bool _is_parsing_svg = false;
    
        /// Vector of IldaFrames currenly available
    std::vector<jam::ilda::IldaFrame> _frames;
    
    std::thread _svg_file_parse_thread;                 // Thread for parsing SVG file asynchronously
    
    
    void _updateFonts() {
        this->_available_fonts.clear();
        CTFontCollectionRef collection = CTFontCollectionCreateFromAvailableFonts(nullptr);
        if (!collection) {
            return;
        }

        CFArrayRef descriptors = CTFontCollectionCreateMatchingFontDescriptors(collection);
        if (!descriptors) {
            CFRelease(collection);
            return;
        }

        CFIndex count = CFArrayGetCount(descriptors);
        for (CFIndex i = 0; i < count; ++i) {
            CTFontDescriptorRef desc = (CTFontDescriptorRef)CFArrayGetValueAtIndex(descriptors, i);

            // Get font file URL
            CFURLRef urlRef = (CFURLRef)CTFontDescriptorCopyAttribute(desc, kCTFontURLAttribute);
            if (!urlRef) continue;

            char path[PATH_MAX];
            if (!CFURLGetFileSystemRepresentation(urlRef, true, (UInt8*)path, sizeof(path))) {
                CFRelease(urlRef);
                continue;
            }

            std::string pathStr(path);
            std::string ext = pathStr.substr(pathStr.find_last_of('.') + 1);
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

            if (ext == "ttf") {
                // Get the display name
                CFStringRef nameRef = (CFStringRef)CTFontDescriptorCopyAttribute(desc, kCTFontDisplayNameAttribute);
                char name[256] = "Unknown";
                if (nameRef) {
                    CFStringGetCString(nameRef, name, sizeof(name), kCFStringEncodingUTF8);
                    CFRelease(nameRef);
                }
                this->_available_fonts[std::string(name)] = pathStr;
            }

            CFRelease(urlRef);
        }

        CFRelease(descriptors);
        CFRelease(collection);
    }
    
        /// Set if the instance currently in the process of importing a file
    void _setParsingStateSvg(bool state) {
        if(state != this->_is_parsing_svg) {
            this->_is_parsing_svg = state;
        }
    }
    
        /// get if the nstance currently in the process of importing a file
    bool _getParsingState() {
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
    int _deNormalizePosition(double pos) {
        int de_normalized = static_cast<int>(pos * 32000);
        return std::clamp(de_normalized, -32767, 32767);
            //        if(pos < 0) {
            //            return static_cast<int>(pos * 32768);
            //        }
            //        return static_cast<int>(pos * 32767);
    };
    
    double _normalizePosition(int pos) {
        return static_cast<double>(pos) / 32000.;
    }
    
        /// adjust the current edit frame index when frame count has changed, to make sure it doen't go out of bounds
    void _updateEditFrame() {
        if(this->_frames.size() == 0) {
            this->_edit_frame = 0;
        } else {
            if(this->_edit_frame > this->_frames.size() - 1) {
                this->_edit_frame = this->_frames.size() - 1;
            }
        }
    }
    
        /// sends out current edit frame, frame count and the file reference
    void _updateOutlets() {
        this->_updateEditFrame();
        atoms msg_atoms;
        queued_message_t msg;
        
        msg_atoms.clear();
        msg_atoms.push_back(this->_frames.size());
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
        jam::ilda::IldaFrame f;
        jam::ilda::IldaHeader h;
        h.setFormatCode(jam::ilda::RecordFormat::FORMAT_5);
        h.setIsColorPallet(false);
        h.setDataRecordCount(0);
        f.setHeader(h);
        this->_frames.push_back(f);
        this->_updateFrameHeaders();
        this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
        this->_edit_frame = this->_frames.size() - 1;
        
        this->_updateOutlets();
    }
    
    void _removeEmptyFrames() {
        for (auto it = this->_frames.begin(); it != this->_frames.end();) {
            auto f = *it;
            if(f.getHeader().getDataRecordCount() == 0) {
                it = this->_frames.erase(it);
            } else {
                it++;
            }
        }
    }
    
        /// update frames in sequens and frame number for all frames
    void _updateFrameHeaders() {
        size_t frame_count = this->_frames.size();
        for(size_t i = 0; i < frame_count; i++) {
            this->_frames[i].getHeader().setFramesInSequence(frame_count);
            this->_frames[i].getHeader().setFrameNumber(i);
            this->_frames[i].getHeader().setCompanyName(this->_company_name);
            this->_frames[i].getHeader().setFrameName(this->_makeFrameName(static_cast<int>(i)));
        }
    }
    
    void _parseFramesToTrueColor(std::vector<jam::ilda::IldaFrame> &frames) {
        jam::ilda::Colors *col = new jam::ilda::Colors();
        
        for(size_t i = 0; i < frames.size(); i++) {
            jam::ilda::IldaHeader h = frames[i].getHeader();
            jam::ilda::RecordFormat rec_format = h.getFormatCode();
            
            switch (rec_format) {
                case jam::ilda::RecordFormat::FORMAT_0:
                    h.setFormatCode(jam::ilda::RecordFormat::FORMAT_4);
                    break;
                case jam::ilda::RecordFormat::FORMAT_1:
                    h.setFormatCode(jam::ilda::RecordFormat::FORMAT_5);
                    break;
                default:
                    continue;;
            }
            frames[i].setHeader(h);
            std::vector<jam::ilda::IldaDataRecord> dr = frames[i].getDataRecords();
            for(size_t j = 0; j < dr.size(); j++) {
                uint8_t color_index = dr[j].getColorIndex();
                std::vector<double> col_vals = col->getFloatColorByIndex((size_t)color_index);
                dr[j].setRed((uint8_t)(col_vals[0] * 255.));
                dr[j].setGreen((uint8_t)(col_vals[1] * 255.));
                dr[j].setBlue((uint8_t)(col_vals[2] * 255.));
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
    VecPoint2D _makeEllipse(
                            Point2D c,
                            Point2D r,
                            const double theta_start = 0,
                            const double theta_end = 360,
                            int segments = 50
                            ) {
        double rad_start = theta_start * (PI / 180);
        double rad_end = theta_end * (PI / 180);
        double rad_range = rad_end - rad_start;
        
        VecPoint2D points;
        for (int i = 0; i <= segments; ++i) {
            Point2D p;
            double angle = rad_start + (rad_range * i / segments);
            p.x = c.x + r.x * std::cos(angle);
            p.y = c.y + r.y * std::sin(angle);
            points.push_back(p);
        }
        return points;
        
    }
    
    void _addDataRecorsToEditFrame(VecPoint2D points, uint8_t r, uint8_t g, uint8_t b) {
        for(size_t i = 0; i < points.size(); i++) {
            jam::ilda::IldaDataRecord dr;
            bool is_blanking = (i == 0);
            uint8_t dr_r = (is_blanking) ? 0 : r;
            uint8_t dr_g = (is_blanking) ? 0 : g;
            uint8_t dr_b = (is_blanking) ? 0 : b;
            
            dr.setRed(dr_r);
            dr.setGreen(dr_g);
            dr.setBlue(dr_b);
            dr.setPosX(this->_deNormalizePosition(points[i].x));
            dr.setPosY(this->_deNormalizePosition(points[i].y));
                // we ondly create/modify 2d data records,but the frame maight be 3d
                // when it was imported from an ILDA file. hence set the y coordinate to 0
            dr.setPosZ(0);
            dr.setBlanking(is_blanking);
            this->_frames[this->_edit_frame].pushRecord(dr);
        }
    };
    
        // TODO: check if nessecary
        /// generate point for circle
        /// @param   c_x                               center coordinate x
        /// @param   c_y                               center coordinate y
        /// @param   r                                   radius
        /// @param   theta_start            start angle in degrees (0º - 360º)
        /// @param   theta_end                 end angle in degrees (0º - 360º)
        /// @param   segments                   number of line segments
    VecPoint2D _makeCircle(
                           const double c_x,
                           const double c_y,
                           const double r,
                           const double theta_start = 0,
                           const double theta_end = 360,
                           int segments = 50
                           ) {
        double rad_start = theta_start * (PI / 180);
        double rad_end = theta_end * (PI / 180);
        double rad_range = rad_end - rad_start;
        
        VecPoint2D points;
        
        for (int i = 0; i <= segments; ++i) {
            Point2D p;
            double angle = rad_start + (rad_range * i / segments);
            p.x = c_x + r * std::cos(angle);
            p.y = c_y + r * std::sin(angle);
            points.push_back(p);
        }
        return points;
    }
    
    
        /// generate point for a rectange with rounded corners
        /// @param   tl                               top left coordinates of the rectange
        /// @param   br                               bottom right coordinates of the rectange
        /// @param   rnd                             corner roundes (0. - 1.) the higer the number the greater the radius of the corner arc
        /// @param   segments                  number of line segments
    VecPoint2D _makeRectangle(
                              Point2D tl,
                              Point2D br,
                              double rnd, // corner roundness
                              int segments = 10
                              ) {
        
        VecPoint2D points;
        Point2D tr = {br.x, tl.y};
        Point2D bl = {tl.x, br.y};
        
            // calculate radius as fraction of shorter rectangle side.
            // the rnd parameter describes the roundness of a corner
            // 0: no rounding, 1: max rounding
            // 1 means we calculaten an arc with the radius on 1/2 of the shorter rectangle side.
        double length_horizontal = abs(tr.x - tl.x);
        double length_vertical = abs(tl.y - bl.y);
        double min_lenght = fmin(length_horizontal, length_vertical);
        double radius = min_lenght * rnd / 2.;
        
            // arc circle radius
        Point2D circle_r = {radius, radius};
        
            // arc top left
        Point2D arc_center_tl = {tl.x + radius, tl.y - radius};
        VecPoint2D arc_tl = this->_makeEllipse(arc_center_tl, circle_r, 180, 90, segments);
        
            // arc top right
        Point2D arc_center_tr = {tr.x - radius, tr.y - radius};
        VecPoint2D arc_tr = this->_makeEllipse(arc_center_tr, circle_r, 90, 0, segments);
        
            // arc bottom right
        Point2D arc_center_br = {br.x - radius, br.y + radius};
        VecPoint2D arc_br = this->_makeEllipse(arc_center_br, circle_r, 0, -90, segments);
        
            // arc bottom left
        Point2D arc_center_bl = {bl.x + radius, bl.y + radius};
        VecPoint2D arc_bl = this->_makeEllipse(arc_center_bl, circle_r, 270, 180, segments);
        
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
    VecPoint2D _makeCubeBezier(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3, int segments = 100) {
        VecPoint2D points;
        for (int i = 0; i <= segments; ++i) {
            double t = static_cast<double>(i) / segments;
            double u = 1.0f - t;
            double x = u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x;
            double y = u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y;
            Point2D p = {x, y};
            points.push_back(p);
        }
        return points;
    };
    
    
    void _rotateFrame(jam::ilda::IldaFrame &f, Point2D c, double angle) {
        angle = -1. * angle;
        double angle_rad = angle * (PI / 180.);
        double cosA      = std::cos(angle_rad);
        double sinA      = std::sin(angle_rad);
        f.reset();
        std::vector<jam::ilda::IldaDataRecord> rotated_records;
        jam::ilda::IldaDataRecord r;
        while(f.getNext(&r)) {
            
            double dx = (double)r.getPosX() - c.x;
            double dy = (double)r.getPosY() - c.y;
            
            double rx = (dx * cosA) - (dy * sinA) + c.x;
            double ry = (dx * sinA) + (dy * cosA) + c.y;
            
            r.setPosX((int)rx);
            r.setPosY((int)ry);
            
            rotated_records.push_back(r);
        }
        
        f.clearRecords();
        for(size_t i = 0; i < rotated_records.size(); i++) {
            f.pushRecord(rotated_records[i]);
        }
    }
    
    void _scaleFrame(jam::ilda::IldaFrame &f, Point2D scale) {
        f.reset();
        std::vector<jam::ilda::IldaDataRecord> scaled_records;
        jam::ilda::IldaDataRecord r;
        while(f.getNext(&r)) {
            r.setPosX(r.getPosX() * scale.x);
            r.setPosY(r.getPosY() * scale.y);
            scaled_records.push_back(r);
        }
        
        f.clearRecords();
        for(size_t i = 0; i < scaled_records.size(); i++) {
            f.pushRecord(scaled_records[i]);
        }
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
    
    MIN_DESCRIPTION     { "Parse SVG file to ILDA file format." };
    MIN_TAGS            { "ILDA, laser tools, utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.ilda.file, jam.jit.gl.ilda.compose"};
//    MIN_FLAGS           {behavior_flags::nobox};
    
    inlet<> input_1             { this, "ILDA file reference", "anything" };
    outlet<> o_file_reference   { this, "ilda file reference"  };
    outlet<> o_font_faces       { this, "Pubulate a umenu with availeble fonts"  };
    outlet<> o_edit_frame       { this, "Frame cureently selected for editing", "int"};
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
                    this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
                    this->_updateOutlets();
                }
                cleaned_args[0] = name;
                return cleaned_args;
            }
        },
        title {"Company Name"},
        description {"Company name set in the headers of the ILDA file.<br />ILDA files contain of a sequence of 'frames'. Every frame has a header summarizing some information about the frame. This attribute sets the value of the header field 'Company Name' (max 8 ASCII characters). "},
        category {"ILDA File"}
    };
    
    attribute<symbol> frameprefix {
        this, "frameprefix", "",
        setter {
            MIN_FUNCTION {
                atoms cleaned_args;
                jam::checkAndFillAttrArgs<std::string>(args, &cleaned_args, 1, "");
                std::string name = static_cast<std::string>(cleaned_args[0]);
                this->_formatIldaString(name, 3);
                this->_frame_name_prefix = name;
                this->_updateFrameHeaders();
                this->_updateFrameHeaders();
                if(this->initialized()) {
                    this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
                    this->_updateOutlets();
                }
                cleaned_args[0] = name;
                return cleaned_args;
            }
        },
        title {"Frame Name Prefix"},
        description {"Frame name prefix set in the header of the ILDA file.<br />ILDA files contain of a sequence of 'frames'. Every frame has a header summarizing some information about the frame. Every frame has a frame-name field in the header. jam.ilda.compose names frames automatically by setting the frame number as its name. This attribute sets an optioname prefix to the frame name (max 3 ASCII characters)."},
        category {"ILDA File"}
    };
    
    message<threadsafe::no> getfonts {
        this, "getfonts", "",
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
    
    message<>bang  {
        this, "bang", "Output ILDA file reference",
        MIN_FUNCTION {
            this->_updateOutlets();
            return {};
        }
    };
    
    message<threadsafe::no> clear {
        this, "clear", "Remove all frames",
        MIN_FUNCTION {
            this->_frames.clear();
            this->_getStructPointer()->clearInstanceFile(this->_instance_id);
            this->_appendEmptyFrame();
            return {};
        }
    };
    
    message<threadsafe::no> seteditframe {
        this, "seteditframe", "Select frame to be edited",
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
            
            if(frame_index < 0 || frame_index > this->_frames.size() - 1) {
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
        this, "geteditframe", "Outputs the currenlty selected frame for editing to the second outlet",
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
        this, "appendframe", "Append a new frame",
        MIN_FUNCTION {
            this->_appendEmptyFrame();
            return {};
        }
    };
    
    message<threadsafe::no> removeframe {
        this, "removeframe", "Remove frame a frame. If no argument is provided the currently set edit frame is removed. If one argument [frame index] is present, the frame at index will be duplicated.",
        MIN_FUNCTION {
            if(this->_frames.size() == 0) {
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
            
            if(frame_index < 0 || frame_index > this->_frames.size() -1) {
                cwarn << "frame index out of range" << endl;
                return {};
            }

            this->_frames.erase(this->_frames.begin() + frame_index);
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            
            if(this->_edit_frame > this->_frames.size() - 1) {
                this->_edit_frame = (this->_frames.size() > 0) ? this->_frames.size() - 1 : 0;
            }
            
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no> duplicateframe {
        this, "duplicateframe", "Duplicate a frame. If no argument is provided the currently set edit frame is duplicated. If one argument [frame index] is present, the frame at index will be duplicated.",
        MIN_FUNCTION {
            if(this->_frames.size() == 0) {
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
            if(frame_index < 0 || frame_index > this->_frames.size() - 1) {
                cwarn << "frame index out of range" << endl;
                return {};
            }
            
            jam::ilda::IldaFrame f = this->_frames[frame_index];
            this->_frames.insert(this->_frames.begin() + frame_index, f);
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
        
    };
    
    message<threadsafe::no> copyframe {
        this, "copyframe", "Copy a frame. The message 'copyframe' followed by two arguments <i>source_index</i> <i>destination_index</i> copies a frame from <i>source_index</i> to <i>destination_index</i>",
        MIN_FUNCTION {
            if(this->_frames.size() == 0) {
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
            
            if(source_index < 0 || source_index > this->_frames.size() - 1 ) {
                cwarn << "source_index out of range" << endl;
                return {};
            }
            
            if(dest_index < 0) {
                cwarn << "destination_index out of range" << endl;
                return {};
            }
    
            
            jam::ilda::IldaFrame f = this->_frames[source_index];
            if(dest_index >= this->_frames.size()) {
                this->_frames.push_back(f);
            } else {
                this->_frames.insert(this->_frames.begin() + dest_index, f);
            }
            
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
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
                if (this->_frames.size() == 0) {
                    this->_appendEmptyFrame();
                }
                double x = args[0];
                double y = args[1];
                jam::ilda::IldaDataRecord r;
                r.setRed(0);
                r.setGreen(0);
                r.setBlue(0);
                r.setPosX(this->_deNormalizePosition(x));
                r.setPosY(this->_deNormalizePosition(y));
                r.setBlanking(true);
                this->_frames[this->_edit_frame].pushRecord(r);
                this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
                
                this->_updateOutlets();
                
                return {};
            }
        }
    };
    
    message<threadsafe::no>svg {
        this, "svg", "Parse a SVG file into the current edit frame.",
        MIN_FUNCTION {
            if(this->_getParsingState()) {
                cwarn << "file loading already in progress" << endl;
                return {};
            }
            
            this->_setParsingStateSvg(true);
            
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
                open_result = c74::max::open_dialog(filename, &path, &outtype, &filetype, (short)1);
                if(open_result != c74::max::MAX_ERR_NONE) {
                    if(open_result < c74::max::MAX_ERR_NONE) {
                        cerr << "couldn't open file" << endl;
                        msg_atoms.clear();
                        msg_atoms.push_back("svg");
                        msg_atoms.push_back(filename);
                        msg_atoms.push_back(0);
                        msg.set(&o_file_result, msg_atoms);
                        msg.send(this);
                    }
                    this->_setParsingStateSvg(false);
                    return{};
                }
            }
            else {
                std::string user_filename = this->_import_args[0];
                if(user_filename.size() > c74::max::MAX_PATH_CHARS - 1) {
                    cerr << "file name too long" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("svg");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                    this->_setParsingStateSvg(false);
                    return {};
                    
                }
                strcpy(filename, user_filename.c_str());
                
                open_result = c74::max::locatefile_extended(filename, &path, &outtype, &filetype, (short)1);
                if(open_result != c74::max::MAX_ERR_NONE) {
                    cerr << "couldn't open file" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("svg");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                    this->_setParsingStateSvg(false);
                    return {};
                }
            }
            
            open_result = c74::max::path_opensysfile( filename, path, &file_handle,c74::max::READ_PERM);
            
            if(open_result != c74::max::MAX_ERR_NONE) {
                cerr << "couldn't open file" << endl;
                msg_atoms.clear();
                msg_atoms.push_back("svg");
                msg_atoms.push_back(filename);
                msg_atoms.push_back(0);
                msg.set(&o_file_result, msg_atoms);
                msg.send(this);
                this->_setParsingStateSvg(false);
                return {};
            }
            
            
            this->_svg_file_parse_thread = std::thread([this]() {
                atoms msg_atoms;
                queued_message_t msg;
                unsigned long size;
                c74::max::t_max_err read_result = 0;
                c74::max::t_handle file_content_handle = nullptr;
                std::string file_content = "";
                c74::max::sysfile_geteof(file_handle,&size);
                
                size = c74::max::sysmem_handlesize(file_content_handle);
                
                if (!(file_content_handle = c74::max::sysmem_newhandle(size))) {
                    cerr << "not enough memory to open " << filename << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("svg");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                    this->_setParsingStateSvg(false);
                    return;
                }
                
                    // https://cycling74.com/forums/t_handle-and-sysmem_newhandle-crash-help-needed
                read_result = c74::max::sysfile_readtextfile(file_handle,file_content_handle,size, c74::max::TEXT_ENCODING_USE_FILE);
                if(read_result != c74::max::MAX_ERR_NONE) {
                    c74::max::sysmem_freehandle(file_content_handle);
                    cerr << "couldn't read file" << endl;
                    msg_atoms.clear();
                    msg_atoms.push_back("svg");
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                    this->_setParsingStateSvg(false);
                    return;
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
                    msg_atoms.push_back(filename);
                    msg_atoms.push_back(0);
                    msg.set(&o_file_result, msg_atoms);
                    msg.send(this);
                    this->_setParsingStateSvg(false);
                    return;
                    
                }
                
                msg_atoms.clear();
                msg_atoms.push_back("svg");
                msg_atoms.push_back(filename);
                msg_atoms.push_back(1);
                msg.set(&o_file_result, msg_atoms);
                msg.send(this);
                this->_setParsingStateSvg(false);
                
                
                    // parse SVG data into shapes
                _shapes.clear();
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
                        this->_shapes.push_back(s);
                    }
                    
                }
                
                nsvgDelete(image);
                
                    // Add shape data to current edit_frame
                if(this->_frames.size() == 0) {
                    this->_appendEmptyFrame();
                }
                
                jam::ilda::IldaFrame f = this->_frames[this->_edit_frame];
                for(size_t i = 0; i< this->_shapes.size(); i++) {
                    for(size_t j = 0; j < this->_shapes[i].getPoints().size(); j++) {
                        bool is_blanking = (j == 0);
                        Point2D p = this->_shapes[i].getPoints()[j];
                        jam::ilda::IldaDataRecord dr;
                        dr.setRed(is_blanking ? 0 : this->_shapes[i].getColor().r);
                        dr.setGreen(is_blanking ? 0 : this->_shapes[i].getColor().g);
                        dr.setBlue(is_blanking ? 0 : this->_shapes[i].getColor().b);
                        dr.setBlanking(is_blanking);
                        dr.setPosX(this->_deNormalizePosition(p.x));
                        dr.setPosY(this->_deNormalizePosition(p.y));
                        f.pushRecord(dr);
                    }
                    
                }
                this->_frames[this->_edit_frame] = f;
                this->_updateFrameHeaders();
                this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
                
                this->_updateOutlets();
                
                
                this->_setParsingStateSvg(false);
            });
            
            this->_svg_file_parse_thread.detach();
            
            

            
            return {};
        }
    };
    
    message<threadsafe::no>font {
        this, "font", "Loads a TTF font face",
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
                    std::string font_path = this->_available_fonts[font_name];
                    short path = 0;
                    short open_result;
                    c74::max::t_fourcc filetype = 'TTF', outtype;
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
                    if(this->_ttfFileProcessor.initFont() != jam::ttf::FontError::NO_ERROR) {
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
        this, "pen", "Set the pen position for wryting text.Arguments:  floats (-1. to 1.) pen_x pen_y",
        MIN_FUNCTION {
            if(args.size() < 2) {
                cwarn << "missing argument for message 'pen'. Expextex two floats" << endl;
                return {};
            }
            this->_pen_pos.x = std::clamp(static_cast<number>(args[0]), -1., 1.);
            this->_pen_pos.y = std::clamp(static_cast<number>(args[1]), -1., 1.);
            return {};
        }
    };
    
    message<threadsafe::no>text {
        this, "text", "Write a text to the current edit frame",
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
            
            VecGlyphPoints points = this->_ttfFileProcessor.getGlyphVertices(in_string, this->_pen_pos);
            if (this->_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            
            for(size_t i = 0; i < points.size(); i++) {
                jam::ilda::IldaDataRecord d_r;
                bool blanking = points[i].type == jam::ttf::VertexType::MoveTo;
                d_r.setRed(this->_color.r * !blanking);
                d_r.setGreen(this->_color.g * !blanking);
                d_r.setBlue(this->_color.b * !blanking);
                d_r.setPosX(this->_deNormalizePosition(points[i].pos.x));
                d_r.setPosY(this->_deNormalizePosition(points[i].pos.y));
                d_r.setBlanking(blanking);
                this->_frames[this->_edit_frame].pushRecord(d_r);
            }

            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            
            this->_updateOutlets();
            
            
            return {};
        }
    };
    
    message<threadsafe::no>line {
        this, "line", "Draw a line into a frame.Arguments: 4 floats (-1. to 1.) start_x start_y end_x end_y",
        MIN_FUNCTION {
            if(args.size() < 4) {
                cwarn << "missing argument for message 'line'" << endl;
                return {};
            }
            if (this->_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            number x_start = args[0];
            number y_start = args[1];
            number x_end   = args[2];
            number y_end   = args[3];
            
           
                // move to staring point
            jam::ilda::IldaDataRecord r_start;
            r_start.setRed(0);
            r_start.setGreen(0);
            r_start.setBlue(0);
            r_start.setPosX(this->_deNormalizePosition(x_start));
            r_start.setPosY(this->_deNormalizePosition(y_start));
            r_start.setBlanking(true);
            
            jam::ilda::IldaDataRecord r_end;
            r_end.setRed(this->_color.r);
            r_end.setGreen(this->_color.g);
            r_end.setBlue(this->_color.b);
            r_end.setPosX(this->_deNormalizePosition(x_end));
            r_end.setPosY(this->_deNormalizePosition(y_end));
            r_end.setBlanking(false);
            
            this->_frames[this->_edit_frame].pushRecord(r_start);
            this->_frames[this->_edit_frame].pushRecord(r_end);
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            
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
            if (this->_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            
                // center point
            Point2D c = {(number)args[0], (number) args[1]};
            
                // radius x/y
            Point2D radius = {(number)args[2], (number)args[2]};
            
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
                //            VecPoint2D points = this->_makeCircle(x, y, radius, t_start, t_end, seg);
            VecPoint2D points = this->_makeEllipse(c, radius, t_start, t_end, seg);
            this->_addDataRecorsToEditFrame(points, this->_color.r, this->_color.g, this->_color.b);
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
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
            if (this->_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
            
                // center point
            Point2D c = {(number)args[0], (number) args[1]};
            
                // radius x/y
            Point2D radius = {(number)args[2], (number)args[3]};
            
            
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
            
            VecPoint2D points = this->_makeEllipse(c, radius, t_start, t_end, seg);
            this->_addDataRecorsToEditFrame(points, this->_color.r, this->_color.g, this->_color.b);
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no>rect {
        this, "rect", "Draw a rectangle into a frame",
        MIN_FUNCTION {
                // rect x_topleft y_topleft x_bottomright y_bottomright r g b corner-radius segment
            if(args.size() < 4) {
                cwarn << "missing argument for message 'rect'" << endl;
                return {};
            }
            if (this->_frames.size() == 0) {
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
            Point2D tl = {tl_x, tl_y};
            Point2D br = {br_x, br_y};
            VecPoint2D points = this->_makeRectangle(tl, br, border_radius);
            this->_addDataRecorsToEditFrame(points, this->_color.r, this->_color.g, this->_color.b);
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
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
            if (this->_frames.size() == 0) {
                this->_appendEmptyFrame();
            }
                // start point
            Point2D start = {(number)args[0], (number) args[1]};
            
                // control point 1
            Point2D c1 = {(number)args[2], (number)args[3]};
            
                // control point 2
            Point2D c2 = {(number)args[4], (number)args[5]};
            
                // end point
            Point2D end = {(number)args[6], (number)args[7]};
            
            
            int seg = 50;
            if(args.size() >= 9) {
                seg = (int)args[8];
                seg = (seg < 3) ? 3 : seg;
                seg = (seg > 200) ? 200 : seg;
            }
            
            VecPoint2D points = this->_makeCubeBezier(start, c1, c2, end, seg);
            this->_addDataRecorsToEditFrame(points, this->_color.r, this->_color.g, this->_color.b);
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no>ilda {
        this, "ilda", "Reference to am ILDA file loaded by [jam.ilda.file]. The frames from the file will be appended. When imported frames use indexed colors, they are converted to true color mode",
        MIN_FUNCTION {
            if(args.size() < 1) {
                cwarn << "missing argument for message 'ilda'" << endl;
                return {};
            }
            std::string ilda_file_refence = args[0];
            std::vector<jam::ilda::IldaFrame> frames = this->_getStructPointer()->getFrames(ilda_file_refence);
            this->_parseFramesToTrueColor(frames);
            if(frames.size() > 0) {
                this->_frames.insert(this->_frames.end(),frames.begin(), frames.end());
            }
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            this->_updateOutlets();
            return {};
        }
    };
    
    message<threadsafe::no> reverseframes {
        this, "reverseframes", "Reverse the order of the frames",
        MIN_FUNCTION {
            std::reverse(this->_frames.begin(), this->_frames.end());
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            this->_updateOutlets();
            return {};
            
        }
    };
    
    message<threadsafe::no>rotateframe {
        this, "rotateframe", "Rotate a frame. If one argument follows the message, the frame will be rotated around the center point. If three arguments are present, the sencond and third arguments specify the rotation center",
        MIN_FUNCTION {
            if(args.size() < 1) {
                cwarn << "missing argument for message 'rotateframe'" << endl;
                return {};
            }
            if (this->_frames.size() == 0) {
                return {};
            }
            number angle = args[0];
            Point2D c = {0., 0.};
            if(args.size() >= 3) {
                c.x = this->_deNormalizePosition((number)args[1]);
                c.y = this->_deNormalizePosition((number)args[2]);
            }
            this->_rotateFrame(this->_frames[this->_edit_frame], c, angle);
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<threadsafe::no>scaleframe {
        this, "scaleframe", "Scale a frame. If one argument follows the message x and y axis are scaled by that value. If 2 arguments are present the first argument specifies the scaling along the x-axis, the second along the y-axis.<br/>Note: All frame modifications are done destructively, meaning information lost e.g. by scaling to 0 cannot be recovered by scaling up again.",
        MIN_FUNCTION {
            if(args.size() < 1) {
                cwarn << "missing argument for message 'scaleframe'" << endl;
                return {};
            }
            if (this->_frames.size() == 0) {
                return {};
            }
            double scale_x = args[0];
            double scale_y = scale_x;
            
            if(args.size() >= 2) {
                scale_y = args[1];
            }
            
            Point2D scale_factors = {scale_x, scale_y};
            this->_scaleFrame(this->_frames[this->_edit_frame], scale_factors);
            this->_updateFrameHeaders();
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            this->_updateOutlets();
            
            return {};
        }
    };
    
    message<>drawcolor {
      this, "drawcolor", "Set the drawing color. <br/>Arguments: 3 floats for red green and blue",
        MIN_FUNCTION {
            if(args.size() < 3) {
                cwarn << "missing argumnet for message 'drawcolor'. Expected 3 floats" << endl;
                return {};
            }
            this->_color.r = static_cast<uint8_t>(std::clamp(static_cast<number>(args[0]), 0., 1.) * 255);
            this->_color.g = static_cast<uint8_t>(std::clamp(static_cast<number>(args[1]), 0., 1.) * 255);
            this->_color.b = static_cast<uint8_t>(std::clamp(static_cast<number>(args[2]), 0., 1.) * 255);
            return {};
        }
    };
    
    message<>export_file {
        this, "export", "Write the frames to ILDA file. If no path/filename is provided, a dialog will be presented. A success/failure notification will be sent to the rightmost outlet in the form export [filename] 0/1.",
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
            this->_getStructPointer()->setInstanceFile(this->_instance_id, this->_frames, std::string(""));
            this->_updateOutlets();
            
            if(err == c74::max::MAX_ERR_NONE) {
                result = jam::ilda::ParseResult::SUCCESS;
                std::vector<unsigned char> file_bytes;
                result = this->_ildaFileProcessor.parseFramesToFileData(file_bytes, this->_frames);
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


