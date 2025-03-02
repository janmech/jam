//
//  ilda_definitions.hpp
//  jam.helios
//
//  Created by Jan Mech on 1/3/25.
//

#ifndef ilda_definitions_h
#define ilda_definitions_h

namespace jam::helios {
    
#define HEADER_MAX_STRING_LENGTH 8
#define HEADER_MAX_RECORD_COUNT 65535
#define HEADER_MAX_FRAME_NUMBER 65535
#define HEADER_MAX_FRAMES_IN_SEQUENCE 65535
#define HEADER_MAX_PROJECTER_NUMBER 65535
    
#define FILE_HEADER_SIZE 32
    
#define FILE_HEADER_ILDA_TAG_START 0
#define FILE_HEADER_ILDA_TAG_LENGTH 4
    
#define FILE_HEADER_RESERVED_START 4
#define FILE_HEADER_RESERVED_LENGTH 3
    
#define FILE_HEADER_FORMAT_CODE_START 7
#define FILE_HEADER_FORMAT_CODE_LENGTH 1
    
#define FILE_HEADER_FRAME_NAME_START 8
#define FILE_HEADER_FRAME_NAME_LENGTH 8
    
#define FILE_HEADER_COMPANY_NAME_START 16
#define FILE_HEADER_COMPANY_NAME_LENGTH 8
    
#define FILE_HEADER_NUMBER_OF_RECODRS_START 24
#define FILE_HEADER_NUMBER_OF_RECODRS_LENGTH 2
    
#define FILE_HEADER_FRAME_NUMBER_START 26
#define FILE_HEADER_FRAME_NUMBER_LENGTH 2
    
#define FILE_HEADER_FRAMES_IN_SEQUENCE_START 28
#define FILE_HEADER_FRAMES_IN_SEQUENCE_LENGTH 2
    
#define FILE_HEADER_PROJECTOR_NUMBER_START 30
#define FILE_HEADER_PROJECTOR_NUMBER_LENGTH 1
    
    enum RecordFormat {
        FORMAT_0 = 0, // 3D Coordinates with Indexed Color
        INDEXED_3D = 0,
        FORMAT_1 = 1, // 2D Coordinates with Indexed Color
        INDEXED_2D = 1,
        FORMAT_2 = 2, // Color Palette
        COLOR_PALLET = 2,
        FORMAT_4 = 4, // 3D Coordinates with True Color
        TRUE_COLOR_3D = 4,
        FORMAT_5 = 5, // 2D Coordinates with True Color
        TRUE_COLOR_2D = 5,
    };
    
    enum ParseResult {
        SUCCESS = 0,
        NODATA = -1,
        PARSEERROR = -2,
        END_OF_FILE = -3,
    };
    
};


#endif /* ilda_definitions_h */
