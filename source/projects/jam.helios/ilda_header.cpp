    //
    //  ilda_header.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 1/3/25.
    //

#include "ilda_header.hpp"

namespace jam::ilda{
    
    RecordFormat Header::getFormatCode() {
        return this->format_code;
    };
    void Header::setFormatCode(RecordFormat format) {
        this->format_code = format;
    };
    
    std::string Header::getFrameName() {
        return this->frame_name;
    };
    
    void Header::setFrameName(std::string frame_name) {
        frame_name.resize(HEADER_MAX_STRING_LENGTH, 0);
        this->frame_name = frame_name;
    };
    
    std::string Header::getCompanyName() {
        return this->company_name;
    };
    
    void Header::setCompanyName(std::string company_name) {
        company_name.resize(HEADER_MAX_STRING_LENGTH, 0);
        this->company_name = company_name;
    };
    
    size_t Header::getRecordCount() {
        return this->record_count;
    };
    
    void Header::setRecordCount(size_t record_count) {
        record_count = (record_count> HEADER_MAX_RECORD_COUNT) ? HEADER_MAX_RECORD_COUNT : record_count;
        this->record_count = record_count;
    };
    
    size_t Header::getFrameNumber() {
        return this->frame_number;
    };
    
    void Header::setFrameNumber(size_t frame_number) {
        this->frame_number = (frame_number > HEADER_MAX_FRAME_NUMBER) ? HEADER_MAX_FRAME_NUMBER : frame_number;
    }
    
    size_t Header::getFramesInSequence() {
        return this->frames_in_sequence;
    };
    
    void Header::setFramesInSequence(size_t frame_count) {
        frame_count = (frame_count> HEADER_MAX_FRAMES_IN_SEQUENCE) ? HEADER_MAX_FRAMES_IN_SEQUENCE : frame_count;
        this->frames_in_sequence = frame_count;
    };
    
    size_t Header::getProjectorNumber() {
        return this->projector_number;
    };
    
    void Header::setProjectorNumber(size_t projector_number) {
        projector_number = (projector_number > HEADER_MAX_PROJECTER_NUMBER) ? HEADER_MAX_PROJECTER_NUMBER : projector_number;
        this->projector_number = projector_number;
    };
    
    bool Header::getIsColorPallet() {
        return this->is_color_pallet;
    };
    
    void Header::setIsColorPallet(bool is_color_pallet) {
        this->is_color_pallet = is_color_pallet;
    };
    
}
