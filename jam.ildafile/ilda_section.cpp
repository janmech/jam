    //
    //  ilds_section.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 2/3/25.
    //

#include "ilda_section.hpp"

namespace jam::ilda {
    void IldaSection::setHeader(IldaHeader header) {
        this->_header = header;
    };
    IldaHeader& IldaSection::getHeader() {
        return this->_header;
    };
    
    void IldaSection::pushRecord(IldaDataRecord data_record) {
        this->_data_records.push_back(data_record);
    };
    
    bool IldaSection::get(IldaDataRecord* data_record, size_t index) {
        if(this->_data_records.size() > index) {
            data_record = &this->_data_records[index];
            return true;
        }
        return false;
        
    };
    
    bool IldaSection::getNext(IldaDataRecord* data_record) {
        if(this->_data_records.size() > this->_iteratorIndex) {
            *data_record = this->_data_records[this->_iteratorIndex];
            this->_iteratorIndex++;
            return true;
        }
        return false;
    };
    
    void IldaSection::IldaSection::reset() {
        this->_iteratorIndex = 0;
    };
};
