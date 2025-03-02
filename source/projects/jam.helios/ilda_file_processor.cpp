    //
    //  ilda_file_processor.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 28/2/25.
    //

#include "ilda_file_processor.hpp"
namespace jam::helios {
    
    /* public functions */
    void IldaFileProcessor::setFileData(std::vector<char>ilda_file) {
        this->_ilda_file = ilda_file;
        this->_fileLoaded = true;
    }
    
    void IldaFileProcessor::clearFileData() {
        this->_ilda_file.clear();
        this->_fileLoaded = false;
    }
    
    ParseResult IldaFileProcessor::parseFileData() {
        ParseResult parse_result = ParseResult::SUCCESS;
        this->_ilda_sections.clear();
        if(!this->fileLoaded()) {
            return ParseResult::NODATA;
        }
        if(this->_ilda_file.size() < FILE_HEADER_SIZE) { // We need at least one complete header
            return ParseResult::PARSEERROR;
        }
        size_t byte_index = 0;
        while(true) {
            // TODO: Continue here. header parsing not yet working
            IldaSection section;
            ParseResult result = this->_extractSection(section, &byte_index);
            this->_ilda_sections.push_back(section);
            if(result != ParseResult::SUCCESS) {
                if(result == ParseResult::END_OF_FILE) {
                    parse_result = ParseResult::SUCCESS;
                    break;
                } else {
                    parse_result = result;
                    break;
                }
               
            }
        }
        
        return parse_result;
    }
    
    bool IldaFileProcessor::fileLoaded() {
        return this->_fileLoaded;
    }
    
    /* protected functions */
    ParseResult IldaFileProcessor::_extractSection(IldaSection &section, size_t *byte_index) {
        IldaHeader section_header;
        section.setHeader(section_header);
            // check start tag
        size_t header_index_start = *byte_index + FILE_HEADER_ILDA_TAG_START;
        char ilda_tag_buffer[FILE_HEADER_ILDA_TAG_LENGTH + 1] = {0};
        this->_readHeaderSection(
                                 ilda_tag_buffer,
                                 (uint8_t)header_index_start,
                                 (uint8_t)FILE_HEADER_ILDA_TAG_LENGTH
                                 );
        std::string start_tag(ilda_tag_buffer);
        if(start_tag != "ILDA") {
            return ParseResult::PARSEERROR;
        }
        
            // read format code
        char format_code_buffer[FILE_HEADER_FORMAT_CODE_LENGTH] = {0};
        header_index_start = *byte_index + FILE_HEADER_FORMAT_CODE_START;
        this->_readHeaderSection(
                                 format_code_buffer,
                                 (uint8_t)header_index_start,
                                 (uint8_t)FILE_HEADER_FORMAT_CODE_LENGTH
                                 );
        int format_code = (int)format_code_buffer[0];
        if(format_code > 5 || format_code == 3) {
            return ParseResult::PARSEERROR;
        }
        section_header.setFormatCode(static_cast<RecordFormat>(format_code));
        
            // read frame name
        char frame_name_buffer[FILE_HEADER_FRAME_NAME_LENGTH + 1] = {0};
        header_index_start = *byte_index + FILE_HEADER_FRAME_NAME_START;
        this->_readHeaderSection(
                                 frame_name_buffer,
                                 (uint8_t)header_index_start,
                                 (uint8_t)FILE_HEADER_FRAME_NAME_LENGTH
                                 );
        section_header.setFrameName(std::string(frame_name_buffer));
        
            // read company name
        char company_name_buffer[FILE_HEADER_COMPANY_NAME_LENGTH + 1] {0};
        header_index_start = *byte_index + FILE_HEADER_COMPANY_NAME_START;
        this->_readHeaderSection(
                                 company_name_buffer,
                                 (uint8_t)header_index_start,
                                 (uint8_t)FILE_HEADER_COMPANY_NAME_LENGTH
                                 );
        section_header.setCompanyName(std::string(company_name_buffer));
        
            // read number of records
        char num_records_buffer[FILE_HEADER_NUMBER_OF_RECODRS_LENGTH] = {0};
        header_index_start = *byte_index + FILE_HEADER_NUMBER_OF_RECODRS_START;
        this->_readHeaderSection(
                                 num_records_buffer,
                                 (uint8_t)header_index_start,
                                 (uint8_t)FILE_HEADER_NUMBER_OF_RECODRS_LENGTH
                                 );
        uint16_t num_record = this->_parseUint16(num_records_buffer, sizeof(num_records_buffer));
        section_header.setRecordCount((size_t)num_record);
        
            // read frame number
        char frame_number_buffer[FILE_HEADER_FRAME_NUMBER_LENGTH] = {0};
        header_index_start = *byte_index + FILE_HEADER_FRAME_NUMBER_START;
        this->_readHeaderSection(
                                 frame_number_buffer,
                                 (uint8_t)header_index_start,
                                 (uint8_t)FILE_HEADER_FRAME_NUMBER_LENGTH
                                 );
        
        uint16_t frame_number = this->_parseUint16(frame_number_buffer, sizeof(frame_number_buffer));
        section_header.setFrameNumber((size_t)frame_number);
        
            // read frames in sequence
        char frames_in_seq_buffer[FILE_HEADER_FRAMES_IN_SEQUENCE_LENGTH] = {};
        header_index_start = *byte_index + FILE_HEADER_FRAMES_IN_SEQUENCE_START;
        this->_readHeaderSection(
                                 frames_in_seq_buffer,
                                 (uint8_t)header_index_start,
                                 (uint8_t)FILE_HEADER_FRAMES_IN_SEQUENCE_LENGTH
                                 );
        uint16_t frames_in_sequence = this->_parseUint16(frames_in_seq_buffer, sizeof(frames_in_seq_buffer));
        section_header.setFramesInSequence((size_t) frames_in_sequence);
        
            // read projector number
        char projector_number_buffer[1] = {0};
        header_index_start = *byte_index + FILE_HEADER_PROJECTOR_NUMBER_START;
        this->_readHeaderSection(
                                 projector_number_buffer,
                                 (uint8_t)header_index_start,
                                 (uint8_t)FILE_HEADER_PROJECTOR_NUMBER_LENGTH
                                 );
        section_header.setProjectorNumber((size_t) projector_number_buffer[0]);
        
        size_t data_records_byte_size = section_header.getRecordCount() * this->_getRecordByteSize(section_header.getFormatCode());
        
        *byte_index = *byte_index + FILE_HEADER_SIZE + data_records_byte_size;
        
        return (*byte_index >= this->_ilda_file.size()) ? ParseResult::END_OF_FILE : ParseResult::SUCCESS;
    };
    
    void IldaFileProcessor::_readHeaderSection(char* buffer, size_t start, size_t byte_count) {
        uint8_t buffer_index = 0;
        size_t end = start + byte_count;
        for(size_t i = start; i < end; i++) {
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
                parsedInt = (uint16_t)bytes[0];
                break;
            default:
                uint8_t most_sig = bytes[0];
                uint8_t least_sig = bytes[1];
                parsedInt = ((uint16_t)most_sig) << 8 | (uint16_t)least_sig;
                break;
        }
        
        return parsedInt;
    }
    
    size_t IldaFileProcessor::_getRecordByteSize(RecordFormat format) {
        switch (format) {
            case RecordFormat::FORMAT_2:
                return 3;
            case RecordFormat::FORMAT_1:
                return 6;
            case RecordFormat::FORMAT_0 :
            case RecordFormat::FORMAT_5 :
                return 8;
            case RecordFormat::FORMAT_4:
                return 10;
            default:
                return 0;
                
        }
    }
};
