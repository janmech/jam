    //
    //  ttf_file_processor.cpp
    //  jam.ilda.compose
    //
    //  Created by Jan Mech on 19/4/25.
    //

#include "ttf_file_processor.hpp"

namespace jam::ttf {
    /// public methods
    void TtfFileProcessor::setFileData(std::vector<uchar>ttf_file) {
        this->clearFileData();
        this->_ttf_file = ttf_file;
    };
    
    void TtfFileProcessor::clearFileData() {
        this->_ttf_file.clear();
        this->_font_initialized = false;
    };
    
    FontError TtfFileProcessor::initFont(int face_index) {
        stbtt_fontinfo font;
        
        int offset = stbtt_GetFontOffsetForIndex(this->_ttf_file.data(), face_index); // face_index comes from descriptor
        if (!stbtt_InitFont(&font, this->_ttf_file.data(), offset)) {
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
        number fontsize,
        const symbol align,
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
        number scale = stbtt_ScaleForPixelHeight(&this->_font, .1 * fontsize);
        
        number text_width = this->_measureTextWidth(text, kerning, scale);
        
        number pen_position_x = pen_pos.x;
        
        number pen_position_y = pen_pos.y - (ascent - line_gap) * scale + ((ascent - descent + line_gap) * scale) / 2.0;

        
        if (align == "center") {
            pen_position_x -= text_width / 2.0;
        } else if (align == "right") {
            pen_position_x -= text_width;
        }
        
        int g_i = 0; // Glyph index
        int prev_g_i = 0; // previous glyph index for kerning
        
        std::string::iterator it = text.begin();
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
                pen_position_x += static_cast<number>(kern) * scale;
//                pen_position_x = pen_position_x + (lsb * scale);
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
                        
                        number x0 = last_point.x;
                        number y0 = last_point.y;
                        number x1 = (v.x * scale) + pen_position_x;
                        number y1 = (v.y * scale) + pen_position_y;
                        number cx = (v.cx * scale) + pen_position_x;
                        number cy = (v.cy * scale) + pen_position_y;
                            // Approximate the quadratic curve using straight lines
                        for (int i = 1; i <= segments; ++i) {
                            number t = static_cast<number>(i) / segments;
                            number u = 1.0f - t;
                            
                            number x = u * u * x0 + 2 * u * t * cx + t * t * x1;
                            number y = u * u * y0 + 2 * u * t * cy + t * t * y1;
                            gv.pos.x = x;
                            gv.pos.y = y;
                            glyph_points.push_back(gv);
                        }
                        last_point.x = gv.pos.x;
                        last_point.y = gv.pos.y;
                    }
                        break;
                    case STBTT_vcubic: { // Cubic bbezier curve flattened to line segments
                        number x0 = last_point.x;
                        number y0 = last_point.y;
                        number x1 = (v.x * scale) + pen_position_x;
                        number y1 = (v.y * scale) + pen_position_y;
                        number cx0 = (v.cx * scale) + pen_position_x;
                        number cy0 = v.cy * scale;
                        number cx1 = (v.cx1 * scale) + pen_position_x;
                        number cy1 = (v.cy1 * scale) + pen_position_y;
                        for (int i = 1; i <= segments; ++i) {
                            number t = static_cast<number>(i) / segments;
                            number u = 1.0f - t;
                            
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
    
    number TtfFileProcessor::getLineHeight(number fontsize) {
        if(!this->_font_initialized) {
            return 0.;
        }
            // Font metrics
        int ascent, descent, line_gap;
        stbtt_GetFontVMetrics(&this->_font, &ascent, &descent, &line_gap);
        
        number scale = stbtt_ScaleForPixelHeight(&this->_font, 0.1 * fontsize);
        return (ascent - descent + line_gap) * scale;
    };
    
    
    /// protected methods
    number TtfFileProcessor::_measureTextWidth(const std::string& text,bool kerning, number scale) {
       
        number width = 0.0;
        std::string::const_iterator it = text.begin();
        int prev_glyph = 0;

        while (it != text.end()) {
            utf8::utfchar32_t codepoint = utf8::next(it, text.end());
            int glyph = stbtt_FindGlyphIndex(&this->_font, codepoint);

            int advance, lsb;
            stbtt_GetGlyphHMetrics(&this->_font, glyph, &advance, &lsb);

            if (kerning && prev_glyph) {
                width += stbtt_GetGlyphKernAdvance(&this->_font, prev_glyph, glyph) * scale;
            }

            width += advance * scale;
            prev_glyph = glyph;
        }

        return width;
    };
    
    
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
