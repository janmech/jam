//
//  ilda_definitions.hpp
//  jam.helios
//
//  Created by Jan Mech on 1/3/25.
//

#ifndef ilda_definitions_h
#define ilda_definitions_h

namespace jam::ilda {
    
#define HEADER_MAX_STRING_LENGTH 8
#define HEADER_MAX_RECORD_COUNT 65535
#define HEADER_MAX_FRAME_NUMBER 65535
#define HEADER_MAX_FRAMES_IN_SEQUENCE 65535
#define HEADER_MAX_PROJECTER_NUMBER 65535
    
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
        TRUE_COLOR_2D = 5
    };
    
};


#endif /* ilda_definitions_h */
