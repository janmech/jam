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
        this->_header.setDataRecordCount(this->_data_records.size());
    };
    
    void IldaFrame::clearRecords() {
        this->_data_records.clear();
    }
    
    bool IldaFrame::get(IldaDataRecord* data_record, size_t index) {
        if(this->_data_records.size() > index) {
            data_record = &this->_data_records[index];
            return true;
        }
        return false;
        
    };
    
    std::vector<IldaDataRecord> IldaFrame::getDataRecords() {
        return this->_data_records;
    };
    
    void IldaFrame::setDataRecords(std::vector<IldaDataRecord> dr) {
        this->_data_records = dr;
    };
    
    bool IldaFrame::getNext(IldaDataRecord* data_record) {
        if(this->_data_records.size() > this->_iteratorIndex) {
            try {
                *data_record = this->_data_records.at(this->_iteratorIndex);
                this->_iteratorIndex++;
                return true;
            } catch (const std::out_of_range& oor) {
                return false;
            }
        }
        return false;
    };
    
    void IldaFrame::IldaFrame::reset() {
        this->_iteratorIndex = 0;
    };
};
