    //
    //  ilda_file_processor.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 28/2/25.
    //

#include "ilda_file_processor.hpp"
namespace jam::ilda {
    
    /* public functions */
    bool IldaFileProcessor::setAndParseIldaFile(std::vector<char>ilda_file) {
        this->_ilda_file = ilda_file;
        if(!this->_parseHeader()) {
            this->clearFileData();
            return false;
        }
        this->_fileLoaded = true;
        return true;
    }
    
    void IldaFileProcessor::clearFileData() {
        this->_ilda_file.clear();
        this->_file_header.companyName="";
        this->_file_header.formatCode = 0;
        this->_file_header.framesInSequence = 0;
        this->_file_header.frameNumber = 0;
        this->_file_header.recordCount = 0;
        this->_file_header.framesInSequence = 0;
        this->_file_header.isColorPallet = false;
        this->_fileLoaded = false;
    }
    
    bool IldaFileProcessor::fileLoaded() {
        return this->_fileLoaded;
    }
    
    section_header_t &IldaFileProcessor::getFileHeader() {
        return this->_file_header;
    }
    
    /* protected functions */
    bool IldaFileProcessor::_parseHeader() {
            // Some basic header evaluation
        if (this->_ilda_file.size() < 32) {
            return false;
        }
        
            // Check the start tag
        char start_tag[5] = {0};
        this->_readHeaderSection(start_tag, FILE_HEADER_ILDA_TAG_START, FILE_HEADER_ILDA_TAG_END);
        std::string start_tag_string(start_tag);
        if(start_tag_string != "ILDA") {
            return false;
        }
        
            // Read the header information
        char format_code;
        this->_readHeaderSection(&format_code, FILE_HEADER_FORMAT_CODE_START, FILE_HEADER_FORMAT_CODE_END);
        if(format_code > 5 || format_code == 3) {
            return false;
        }
        
        char frame_name[10] = {0};
        this->_readHeaderSection(frame_name, FILE_HEADER_FRAME_NAME_START, FILE_HEADER_FRAME_NAME_END);
        
        char company_name[10] = {0};
        this->_readHeaderSection(company_name, FILE_HEADER_COMPANY_NAME_START, FILE_HEADER_COMPANY_NAME_END);
        
            // For color palettes, the number of records SHALL be between 2 and 256. A color pallet is indicateb by frames in sequence == 0
        char record_count[2] = {};
        this->_readHeaderSection(record_count, FILE_HEADER_NUMBER_OF_RECODRS_START, FILE_HEADER_NUMBER_OF_RECODRS_END);
            // If the number of records is 0, then this is to be taken as the end of file header and no more data will follow this header.
        if(this->_parseUint16(record_count, sizeof(record_count)) == 0) {
            return false;
        }
        
            // If the frame is part of a group such as an animation sequence, this represents the frame number. Counting begins with frame 0. Range is 0 – 65534. TODO: Is this correct or a type in the specs? (0xFFFF == 65535)
        char frame_number[2] = {0};
        this->_readHeaderSection(frame_number, FILE_HEADER_FRAME_NUMBER_START, FILE_HEADER_FRAME_NUMBER_END);
        if(this->_parseUint16(frame_number, sizeof(frame_number)) > 65534) {
            return false;
        }
        
            // Total frames in this group or sequence. Range is 1 – 65535. For color palettes this SHALL be 0.
        char frames_in_sequence[2] = {0};
        this->_readHeaderSection(frames_in_sequence, FILE_HEADER_FRAMES_IN_SEQUENCE_START, FILE_HEADER_FRAMES_IN_SEQUENCE_END);
        
        
        this->_file_header.formatCode = format_code;
        this->_file_header.frameName = std::string(frame_name);
        this->_file_header.companyName = std::string(company_name);
        this->_file_header.recordCount = this->_parseUint16(record_count, sizeof(record_count));
        this->_file_header.frameNumber = this->_parseUint16(frame_number, sizeof(frame_number));
        this->_file_header.framesInSequence = this->_parseUint16(frames_in_sequence, sizeof(frames_in_sequence));
        this->_file_header.isColorPallet = (this->_file_header.framesInSequence == 0);
        
            // finally validate if record count is in bounds for color pallet
        if(this->_file_header.isColorPallet) {
            if(this->_file_header.recordCount < 2 || this->_file_header.recordCount > 256) {
                return false;
            }
        }
        
        return true;
    }
    
    void IldaFileProcessor::_readHeaderSection(char* buffer, uint8_t start, uint8_t end) {
        uint8_t buffer_index = 0;
        for(size_t i = start; i <= end; i++) {
            buffer[buffer_index] = this->_ilda_file[i];
            buffer_index++;
        }
    }
    
    std::uint16_t IldaFileProcessor::_parseUint16(char* bytes, std::size_t byte_count) {
        uint16_t parsedInt = 0;
        switch (byte_count) {
            case 0:
                parsedInt = 0;
                break;
            case 1:
                parsedInt = (std::uint16_t)bytes[0];
                break;
            default:
                parsedInt = (((std::uint16_t)bytes[0]) << 8) | (uint16_t)bytes[1];
                break;
        }
        
        return parsedInt;
    }
};
