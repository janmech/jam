    //
    //  ilda_frame.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 2/3/25.
    //

#include "ilda_frame.hpp"

namespace jam::ilda {
    void IldaFrame::setHeader(IldaHeader header) {
        this->_header = header;
    };
    IldaHeader& IldaFrame::getHeader() {
        return this->_header;
    };
    
    void IldaFrame::pushRecord(IldaDataRecord data_record) {
        this->_data_records.push_back(data_record);
    };
    
    bool IldaFrame::get(IldaDataRecord* data_record, size_t index) {
        if(this->_data_records.size() > index) {
            data_record = &this->_data_records[index];
            return true;
        }
        return false;
        
    };
    
    bool IldaFrame::getNext(IldaDataRecord* data_record) {
        if(this->_data_records.size() > this->_iteratorIndex) {
            *data_record = this->_data_records[this->_iteratorIndex];
            this->_iteratorIndex++;
            return true;
        }
        return false;
    };
    
    void IldaFrame::IldaFrame::reset() {
        this->_iteratorIndex = 0;
    };
};
