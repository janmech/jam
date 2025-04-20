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
    
    std::vector<GlyphVertex> TtfFileProcessor::getGlyphVertices(std::string text, double height, int segments) {
        
        
        std::vector<GlyphVertex> glyph_points;
        if(!this->_font_initialized) {
            return glyph_points;
        }
        
            // Font metrics
        int ascent, descent, lineGap;
        stbtt_GetFontVMetrics(&this->_font, &ascent, &descent, &lineGap);
        
            // Font scaling factor.
        double scale = stbtt_ScaleForPixelHeight(&this->_font, .5);
        
        
        std::string::iterator it = text.begin();
        double pen_position = 0.;
        int g_i = 0; // Glyph index
        int prev_g_i = 0; // previous glyph index for kerning
        while (it != text.end()) {
            
            utf8::utfchar32_t codepoint = utf8::next(it, text.end()); // get the font glyph index from unicode character
            g_i = stbtt_FindGlyphIndex(&this->_font, codepoint);
        
                //        int codepoint = this->_getCodepointFromUTF8(c);
            if (codepoint != -1) {
                g_i = stbtt_FindGlyphIndex(&this->_font, codepoint);
            }
            
//            if (prev_g_i) {x
//                int kern = stbtt_GetGlyphKernAdvance(&this->_font, prev_g_i, g_i);
//                pen_position += static_cast<double>(kern) * scale;
//            }
            
                // Get advance width and left-side bearing
            int ad_w, lsb;
            stbtt_GetGlyphHMetrics(&this->_font, g_i, &ad_w, &lsb);
            
                // Glyph metrics
            int box_x0, box_y0, box_x1, box_y1;
            stbtt_GetGlyphBox(&this->_font, g_i, &box_x0, &box_y0, &box_x1, &box_y1);
            
            double g_width = (box_x1 - box_x0) * scale;
            
            
            stbtt_vertex* vertices = nullptr;
            int num_verts = stbtt_GetGlyphShape(&this->_font, g_i, &vertices);
            
            Point2D last_point = {0., 0.};
            
            for (int i = 0; i < num_verts; ++i) {
                stbtt_vertex& v = vertices[i];
                
                GlyphVertex gv;
                
                switch (v.type) {
                    case STBTT_vmove: {
                        gv.type = VertexType::MoveTo;
                        gv.pos.x = v.x * scale;
                        gv.pos.y = v.y * scale;
                        last_point.x = gv.pos.x;
                        last_point.y = gv.pos.y;
                        glyph_points.push_back(gv);
                    }
                        break;
                    case STBTT_vline: {
                        gv.type = VertexType::LineTo;
                        gv.pos.x = v.x * scale;
                        gv.pos.y = v.y * scale;
                        last_point.x = gv.pos.x;
                        last_point.y = gv.pos.y;
                        glyph_points.push_back(gv);
                    }
                        break;
                    case STBTT_vcurve: {
                        gv.type = VertexType::CurveTo;
                        
                        double x0 = last_point.x;
                        double y0 = last_point.y;
                        double x1 = v.x * scale;
                        double y1 = v.y * scale;
                        double cx = v.cx * scale;
                        double cy = v.cy * scale;
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
                    case STBTT_vcubic: {
                        double x0 = last_point.x;
                        double y0 = last_point.y;
                        double x1 = v.x * scale;
                        double y1 = v.y * scale;
                        double cx0 = v.cx * scale;
                        double cy0 = v.cy * scale;
                        double cx1 = v.cx1 * scale;
                        double cy1 = v.cy1 * scale;
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
            
//            double glyph_x = pen_position + (lsb * scale);
            double glyph_x = pen_position;
                //            double glyph_yY = baselineY;
            for(size_t i = 0; i < glyph_points.size(); i++) {
                    // adjust to x coordinates -1. to 1.
                glyph_points[i].pos.x = glyph_points[i].pos.x - 1.;
                
                    // add left-side bearing and pen position
                glyph_points[i].pos.x = glyph_points[i].pos.x + pen_position;
            }
            pen_position += ad_w / 2. * scale ;
//            pen_position += g_width;
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
