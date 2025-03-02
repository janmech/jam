    //
    //  ilds_section.hpp
    //  jam.helios
    //
    //  Created by Jan Mech on 2/3/25.
    //

#ifndef ilds_section_hpp
#define ilds_section_hpp

#include <stdio.h>
#include <vector>
#include "ilda_definitions.hpp"
#include "ilda_header.hpp"
#include "ilda_data_record.hpp"

namespace jam::helios {
    class IldaSection {
    public:
        IldaSection(){};
        ~IldaSection(){};
        
        void setHeader(IldaHeader header);
        IldaHeader& getHeader();
        
        void pushRecord(IldaDataRecord data_record);
        
        bool get(IldaDataRecord* data_record, size_t index);
        bool getNext(IldaDataRecord* data_record);
        void reset();
        
        
        
        
    protected:
        size_t _iteratorIndex = 0;
        IldaHeader _header;
        std::vector<IldaDataRecord> _data_records;
        
    };
};

#endif /* ilds_section_hpp */
