//
//  ttf_file_processor.hpp
//  jam.ilda.compose
//
//  Created by Jan Mech on 19/4/25.
//

#ifndef ttf_file_processor_hpp
#define ttf_file_processor_hpp

#include <iostream>
#include <fstream>
#include <vector>
#include "stb_truetype.h"
#include "../jam.ilda_common/utfcpp/source/utf8.h"

namespace jam::ttf {
    using uchar = unsigned char;
    
    enum FontError {
        NO_ERROR = 0,
        FONT_INIT_ERROR = -1
    };
    
    struct Point2D {
        double x;
        double y;
    };

    enum class VertexType {
        MoveTo,
        LineTo,
        CurveTo,
        CubicTo
    };
    
    struct GlyphVertex {
        VertexType type = VertexType::LineTo;
        Point2D pos = {0., 0.};
    };
    
    class TtfFileProcessor {
        
    protected:
        std::vector<uchar> _ttf_file;
        
        stbtt_fontinfo _font;
        
        bool _font_initialized = false;
        
        int _getCodepointFromUTF8(const std::string& utf8);

        
    public:
        TtfFileProcessor() = default;
        TtfFileProcessor(std::vector<uchar> ttf_file) {
            this->_ttf_file = ttf_file;
        }
        
        void setFileData(std::vector<uchar>ttf_file);
        void clearFileData();
        FontError initFont();
        bool fontInitialized();
        std::vector<GlyphVertex> getGlyphVertices(std::string c, double height = 1.f, int segments = 10);
        
        
    
        
    };
}

#endif /* ttf_file_processor_hpp */
