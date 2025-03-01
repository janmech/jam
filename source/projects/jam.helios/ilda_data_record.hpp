    //
    //  ilda_data_record.hpp
    //  jam.helios
    //
    //  Created by Jan Mech on 1/3/25.
    //

#ifndef ilda_data_record_hpp
#define ilda_data_record_hpp

#include <stdio.h>
#include <cstddef>
#include "ilda_definitions.hpp"
/**
 TODO: continue here
 section:
     header:
     data-records[]


 data record class:
     format,
     x, y, z, (int)
     bool: last point
     bool: blanking
     color index
     R,G,B


 file vector:
     array of sections
 
 
 Format 0
     3D Coordinates with Indexed Color

     8 Bytes :
         x (2)
         y (2)
         z (2)
         status code (1)
         color index (1)


 Format 1
     2D Coordinates with Indexed Color
     6 Bytes :
         x (2)
         y (2)
         status code (1)
         color index (1)

 Format 2
     Color Palette
     3 Bytes:
         R (1)
         G (1)
         B (1)

Format 4
    3D Coordinates with True Color
    10 Bytes:
        x (2)
         y (2)
         z (2)
         status code (1)
         R (1)
         G (1)
         B (1)

 Format 5
     2D Coordinates with True Color
     8 Bytes:
     x (2)
     y (2)
     status code (1)
     R (1)
     G (1)
     B (1)

 */

namespace jam::ilda {
    class DataRecord {
    public:
        
        DataRecord(Header &section_header): sectionHeader(section_header){};
        
        RecordFormat getFormat(){
            return this->sectionHeader.getFormatCode();
        }
        
    protected:
        Header &sectionHeader;
      
        
    };
};
    
#endif /* ilda_data_record_hpp */
