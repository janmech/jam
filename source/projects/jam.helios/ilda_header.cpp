    //
    //  ilda_header.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 1/3/25.
    //

#include "ilda_header.hpp"

namespace jam::helios{
    
    RecordFormat IldaHeader::getFormatCode() {
        return this->format_code;
    };
    
    std::string IldaHeader::getFormat() {
        std::string format = "";
        switch (this->getFormatCode()) {
            case RecordFormat::FORMAT_0 :
                format = "3D Coordinates with Indexed Color";
                break;
            case RecordFormat::FORMAT_1 :
                format = "2D Coordinates with Indexed Color";
                break;
            case RecordFormat::FORMAT_2:
                format = "Color Palette";
                break;
            case RecordFormat::FORMAT_4:
                format = "3D Coordinates with True Color";
                break;
            case RecordFormat::FORMAT_5:
                format = "2D Coordinates with True Colorr";
                break;
            default:
                format = "Not recognized";
                break;
        }
        return format;
    };
    void IldaHeader::setFormatCode(RecordFormat format) {
        this->format_code = format;
    };
    
    std::string IldaHeader::getFrameName() {
        return this->frame_name;
    };
    
    void IldaHeader::setFrameName(std::string frame_name) {
        frame_name.resize(HEADER_MAX_STRING_LENGTH, 0);
        this->frame_name = frame_name;
    };
    
    std::string IldaHeader::getCompanyName() {
        return this->company_name;
    };
    
    void IldaHeader::setCompanyName(std::string company_name) {
        company_name.resize(HEADER_MAX_STRING_LENGTH, 0);
        this->company_name = company_name;
    };
    
    size_t IldaHeader::getRecordCount() {
        return this->record_count;
    };
    
    void IldaHeader::setRecordCount(size_t record_count) {
        record_count = (record_count> HEADER_MAX_RECORD_COUNT) ? HEADER_MAX_RECORD_COUNT : record_count;
        this->record_count = record_count;
    };
    
    size_t IldaHeader::getFrameNumber() {
        return this->frame_number;
    };
    
    void IldaHeader::setFrameNumber(size_t frame_number) {
        this->frame_number = (frame_number > HEADER_MAX_FRAME_NUMBER) ? HEADER_MAX_FRAME_NUMBER : frame_number;
    }
    
    size_t IldaHeader::getFramesInSequence() {
        return this->frames_in_sequence;
    };
    
    void IldaHeader::setFramesInSequence(size_t frame_count) {
        frame_count = (frame_count> HEADER_MAX_FRAMES_IN_SEQUENCE) ? HEADER_MAX_FRAMES_IN_SEQUENCE : frame_count;
        this->frames_in_sequence = frame_count;
    };
    
    size_t IldaHeader::getProjectorNumber() {
        return this->projector_number;
    };
    
    void IldaHeader::setProjectorNumber(size_t projector_number) {
        projector_number = (projector_number > HEADER_MAX_PROJECTER_NUMBER) ? HEADER_MAX_PROJECTER_NUMBER : projector_number;
        this->projector_number = projector_number;
    };
    
    bool IldaHeader::getIsColorPallet() {
        return this->is_color_pallet;
    };
    
    void IldaHeader::setIsColorPallet(bool is_color_pallet) {
        this->is_color_pallet = is_color_pallet;
    };
    
}
