    //
    //  ilda_header.hpp
    //  jam.helios
    //
    //  Created by Jan Mech on 1/3/25.
    //

#ifndef ilda_header_hpp
#define ilda_header_hpp

#include <stdio.h>
#include <cstddef>
#include <string>
#include "ilda_definitions.hpp"

namespace jam::ilda {
    
//#define HEADER_MAX_STRING_LENGTH 8
//#define HEADER_MAX_RECORD_COUNT 65535
//#define HEADER_MAX_FRAME_NUMBER 65535
//#define HEADER_MAX_FRAMES_IN_SEQUENCE 65535
//#define HEADER_MAX_PROJECTER_NUMBER 65535
    
//    enum RecordFormat {
//        FORMAT_0 = 0, // 3D Coordinates with Indexed Color
//        INDEXED_3D = 0,
//        FORMAT_1 = 1, // 2D Coordinates with Indexed Color
//        INDEXED_2D = 1,
//        FORMAT_2 = 2, // Color Palette
//        COLOR_PALLET = 2,
//        FORMAT_4 = 4, // 3D Coordinates with True Color
//        TRUE_COLOR_3D = 4,
//        FORMAT_5 = 5, // 2D Coordinates with True Color
//        TRUE_COLOR_2D = 5
//    };
//    
    class Header {
    public:
        Header(){};
        
        Header(
               RecordFormat format_code, std::string frame_name = "", std::string company_name = "",
               size_t record_count = 0, size_t frame_number = 0, size_t frames_in_sequence = 0,
               size_t projector_number = 0, bool is_color_pallet = false
               ){
            // We are using the setters here instead of init list to ensure correct value ranges
            this->setFormatCode(format_code);
            this->setFrameName(frame_name);
            this->setCompanyName(company_name);
            this->setRecordCount(record_count);
            this->setFrameNumber(frame_number);
            this->setFramesInSequence(frames_in_sequence);
            this->setProjectorNumber(projector_number);
            this->setIsColorPallet(is_color_pallet);
            
            
        };
        
        ~Header(){};
        
        
        RecordFormat getFormatCode();
        void setFormatCode(RecordFormat format);
        
        std::string getFrameName();
        void setFrameName(std::string frame_name);
        
        std::string getCompanyName();
        void setCompanyName(std::string company_name);
        
        size_t getRecordCount();
        void setRecordCount(size_t record_count);
        
        size_t getFrameNumber();
        void setFrameNumber(size_t frame_number);
        
        size_t getFramesInSequence();
        void setFramesInSequence(size_t frame_count);
        
        size_t getProjectorNumber();
        void setProjectorNumber(size_t projector_number);
        
        bool getIsColorPallet();
        void setIsColorPallet(bool is_color_pallet);
        
        
    protected:
        RecordFormat format_code = RecordFormat::INDEXED_3D ;
        std::string frame_name = "";
        std::string company_name = "";
        size_t record_count = 0;
        size_t frame_number = 0;
        size_t frames_in_sequence = 0;
        size_t projector_number = 0;
        bool is_color_pallet = false;
        
    };
};

#endif /* ilda_header_hpp */
