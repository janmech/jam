    //
    //  ttf_file_processor.cpp
    //  jam.ilda.compose
    //
    //  Created by Jan Mech on 19/4/25.
    //

#include "ttf_file_processor.hpp"

namespace jam::ttf {
    void TtfFileProcessor::setFileData(std::vector<uchar>ttf_file) {
        this->clearFileData();
        this->_ttf_file = ttf_file;
    };
    
    void TtfFileProcessor::clearFileData() {
        this->_ttf_file.clear();
        this->_font_initialized = false;
    };
    
    FontError TtfFileProcessor::initFont() {
        stbtt_fontinfo font;
        if (!stbtt_InitFont(&font, this->_ttf_file.data(), 0)) {
            this->_font_initialized = false;
            return FontError::FONT_INIT_ERROR;
        }
        this->_font = font;
        this->_font_initialized = true;
        return FontError::NO_ERROR;
    };
    
    
    bool TtfFileProcessor::fontInitialized() {
        return this->_font_initialized;
    };
    
    std::vector<GlyphVertex> TtfFileProcessor::getGlyphVertices(
        std::string text,
        Point2D pen_pos,
        bool kerning,
        double fontsize,
        int segments
    ) {
        std::vector<GlyphVertex> glyph_points;
        if(!this->_font_initialized) {
            return glyph_points;
        }
        
            // Font metrics
        int ascent, descent, line_gap;
        stbtt_GetFontVMetrics(&this->_font, &ascent, &descent, &line_gap);
        
            // Font scaling factor.
        double scale = stbtt_ScaleForPixelHeight(&this->_font, .1 * fontsize);
        
        
        std::string::iterator it = text.begin();
        double pen_position_x = pen_pos.x;
        double pen_position_y = pen_pos.y - ((ascent - line_gap) * scale);
        int g_i = 0; // Glyph index
        int prev_g_i = 0; // previous glyph index for kerning
        while (it != text.end()) {
            utf8::utfchar32_t codepoint = utf8::next(it, text.end()); // get the font glyph index from unicode character
            g_i = stbtt_FindGlyphIndex(&this->_font, codepoint);
        
                //        int codepoint = this->_getCodepointFromUTF8(c);
            if (codepoint != -1) {
                g_i = stbtt_FindGlyphIndex(&this->_font, codepoint);
            }
            
                // Get advance width and left-side bearing
            int ad_w, lsb;
            stbtt_GetGlyphHMetrics(&this->_font, g_i, &ad_w, &lsb);
            
                // Add kerning and left-side bearing if it is not the first character.
            if (prev_g_i && kerning) {
                int kern = stbtt_GetGlyphKernAdvance(&this->_font, prev_g_i, g_i);
                pen_position_x += static_cast<double>(kern) * scale;
                pen_position_x = pen_position_x + (lsb * scale);
            }
            
                // Get glypf vertices and flatten them to line segments
            stbtt_vertex* vertices = nullptr;
            int num_verts = stbtt_GetGlyphShape(&this->_font, g_i, &vertices);
            
            Point2D last_point = {0., 0.};
            
            for (int i = 0; i < num_verts; ++i) {
                stbtt_vertex& v = vertices[i];
                
                GlyphVertex gv;
                
                switch (v.type) {
                    case STBTT_vmove: {
                        gv.type = VertexType::MoveTo;
                        gv.pos.x = (v.x * scale) + pen_position_x;
                        gv.pos.y = (v.y * scale) + pen_position_y;
                        last_point.x = gv.pos.x;
                        last_point.y = gv.pos.y;
                        glyph_points.push_back(gv);
                    }
                        break;
                    case STBTT_vline: {
                        gv.type = VertexType::LineTo;
                        gv.pos.x = (v.x * scale) + pen_position_x;
                        gv.pos.y = (v.y * scale) + pen_position_y;
                        last_point.x = gv.pos.x;
                        last_point.y = gv.pos.y;
                        glyph_points.push_back(gv);
                    }
                        break;
                    case STBTT_vcurve: { // Qudradic bezier curve flattened to line segments
                        gv.type = VertexType::CurveTo;
                        
                        double x0 = last_point.x;
                        double y0 = last_point.y;
                        double x1 = (v.x * scale) + pen_position_x;
                        double y1 = (v.y * scale) + pen_position_y;
                        double cx = (v.cx * scale) + pen_position_x;
                        double cy = (v.cy * scale) + pen_position_y;
                            // Approximate the quadratic curve using straight lines
                        for (int i = 1; i <= segments; ++i) {
                            double t = static_cast<double>(i) / segments;
                            double u = 1.0f - t;
                            
                            double x = u * u * x0 + 2 * u * t * cx + t * t * x1;
                            double y = u * u * y0 + 2 * u * t * cy + t * t * y1;
                            gv.pos.x = x;
                            gv.pos.y = y;
                            glyph_points.push_back(gv);
                        }
                        last_point.x = gv.pos.x;
                        last_point.y = gv.pos.y;
                    }
                        break;
                    case STBTT_vcubic: { // Cubic bbezier curve flattened to line segments
                        double x0 = last_point.x;
                        double y0 = last_point.y;
                        double x1 = (v.x * scale) + pen_position_x;
                        double y1 = (v.y * scale) + pen_position_y;
                        double cx0 = (v.cx * scale) + pen_position_x;
                        double cy0 = v.cy * scale;
                        double cx1 = (v.cx1 * scale) + pen_position_x;
                        double cy1 = (v.cy1 * scale) + pen_position_y;
                        for (int i = 1; i <= segments; ++i) {
                            double t = static_cast<double>(i) / segments;
                            double u = 1.0f - t;
                            
                            float x = u * u * u * x0 + 3. * u * u * t * cx0 + 3. * u * t * t * cx1 + t * t * t * x1;
                            float y = u * u * u * y0 + 3. * u * u * t * cy0 + 3. * u * t * t * cy1 + t * t * t * y1;
                            
                            gv.pos.x = x;
                            gv.pos.y = y;
                            glyph_points.push_back(gv);
                        }
                        
                        last_point.x = gv.pos.x;
                        last_point.y = gv.pos.y;
                        
                    }
                        break;
                }
            }
            pen_position_x += ad_w * scale; // Move the pen forward
            prev_g_i = g_i;
            stbtt_FreeShape(&this->_font, vertices);
            
        }
        
        return glyph_points;
    }
    
    
    int TtfFileProcessor::_getCodepointFromUTF8(const std::string& utf8) {
        const unsigned char* s = reinterpret_cast<const unsigned char*>(utf8.c_str());
        
        if (s[0] < 0x80) {
            return s[0]; // ASCII
        } else if ((s[0] & 0xE0) == 0xC0) {
            return ((s[0] & 0x1F) << 6) | (s[1] & 0x3F);
        } else if ((s[0] & 0xF0) == 0xE0) {
            return ((s[0] & 0x0F) << 12) |
            ((s[1] & 0x3F) << 6) |
            (s[2] & 0x3F);
        } else if ((s[0] & 0xF8) == 0xF0) {
            return ((s[0] & 0x07) << 18) |
            ((s[1] & 0x3F) << 12) |
            ((s[2] & 0x3F) << 6) |
            (s[3] & 0x3F);
        }
        
        return -1; // Invalid or unsupported UTF-8
    };
    
    
    
}
