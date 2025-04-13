    //
    //  ilds_frame.hpp
    //  jam.helios
    //
    //  Created by Jan Mech on 2/3/25.
    //

#ifndef ilda_frame_hpp
#define ilda_frame_hpp

#include <stdio.h>
#include <vector>
#include "ilda_definitions.hpp"
#include "ilda_header.hpp"
#include "ilda_data_record.hpp"

namespace jam::ilda {
    class IldaFrame {
    public:
        IldaFrame(){};
        ~IldaFrame(){};
        
        void setHeader(IldaHeader header);
        IldaHeader& getHeader();
        
        void pushRecord(IldaDataRecord data_record);
        
        void clearRecords();
        
        /// get  the data records vector
        std::vector<IldaDataRecord> getDataRecords();
        
        void setDataRecords(std::vector<IldaDataRecord> dr);
        
        bool get(IldaDataRecord* data_record, size_t index);
        
        /// convenience iterarator to read out data records.
        bool getNext(IldaDataRecord* data_record);
        /// reset the internal data record iterator to first element
        void reset();
        
        
        
        
    protected:
        size_t _iteratorIndex = 0;
        IldaHeader _header;
        std::vector<IldaDataRecord> _data_records;
        
    };
};

#endif /* ilda_frame_hpp */
