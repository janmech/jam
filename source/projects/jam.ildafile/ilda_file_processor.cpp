    //
    //  ilda_file_processor.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 28/2/25.
    //

#include "ilda_file_processor.hpp"
namespace jam::helios {
    
    /* public methods */
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
            this->clearFileData();
            return ParseResult::ERROR;
        }
        size_t byte_index = 0;
        while(true) {
            IldaSection section;
            ParseResult result = this->_extractSection(section, &byte_index);
            this->_ilda_sections.push_back(std::move(section));
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
    
    std::vector<IldaSection> IldaFileProcessor::getSections() {
        return this->_ilda_sections;
    }
    
    /* protected methods */
    ParseResult IldaFileProcessor::_extractSection(IldaSection &section, size_t *byte_index) {
        IldaHeader section_header;
        ParseResult header_parse_result = this->_parseSectionHeader(section_header, byte_index);
        if(header_parse_result != ParseResult::SUCCESS) {
            this->clearFileData();
            return header_parse_result;
        }
        size_t record_count = section_header.getDataRecordCount();
        for (size_t record_index = 0; record_index < record_count; record_index++) {
            size_t record_byte_size = this->_getRecordByteSize(section_header.getFormatCode());
            size_t record_start_index = *byte_index + FILE_HEADER_SIZE + (record_index * record_byte_size);
            char record_buffer[10] = {0};
            this->_getFileBytes(
                                record_buffer,
                                record_start_index,
                                record_byte_size
                                );
            ParseResult record_parse_result = ParseResult::SUCCESS;
            IldaDataRecord data_record;
            switch (section_header.getFormatCode()) {
                case RecordFormat::FORMAT_0 :
                    record_parse_result = this->_parseDataRecordFormat_0(data_record, record_buffer);
                    break;
                case RecordFormat::FORMAT_1 :
                    record_parse_result = this->_parseDataRecordFormat_1(data_record, record_buffer);
                    break;
                case RecordFormat::FORMAT_2 :
                    record_parse_result = this->_parseDataRecordFormat_2(data_record, record_buffer);
                    break;
                case RecordFormat::FORMAT_4 :
                    record_parse_result = this->_parseDataRecordFormat_4(data_record, record_buffer);
                    break;
                case RecordFormat::FORMAT_5 :
                    record_parse_result = this->_parseDataRecordFormat_5(data_record, record_buffer);
                    break;
                default:
                    record_parse_result = ParseResult::ERROR;
            }
            if(record_parse_result == ParseResult::ERROR) {
                return record_parse_result;
            }
            section.pushRecord(std::move(data_record));
        }
        
        
        size_t data_records_byte_size = section_header.getDataRecordCount() * this->_getRecordByteSize(section_header.getFormatCode());
        
        *byte_index = *byte_index + FILE_HEADER_SIZE + data_records_byte_size;
        section.setHeader(std::move(section_header));
        
        return (*byte_index >= this->_ilda_file.size()) ? ParseResult::END_OF_FILE : ParseResult::SUCCESS;
    };
    
    ParseResult IldaFileProcessor::_parseSectionHeader(IldaHeader &section_header, size_t *byte_index) {
            // check start tag
        size_t header_start_index = *(byte_index) + FILE_HEADER_ILDA_TAG_START;
        char ilda_tag_buffer[FILE_HEADER_ILDA_TAG_LENGTH + 1] = {0};
        this->_getFileBytes(
                            ilda_tag_buffer,
                            header_start_index,
                            FILE_HEADER_ILDA_TAG_LENGTH
                            );
        std::string start_tag(ilda_tag_buffer);
        if(start_tag != "ILDA") {
            return ParseResult::ERROR;
        }
        
            // read format code
        char format_code_buffer[FILE_HEADER_FORMAT_CODE_LENGTH] = {0};
        header_start_index = *byte_index + FILE_HEADER_FORMAT_CODE_START;
        this->_getFileBytes(
                            format_code_buffer,
                            header_start_index,
                            FILE_HEADER_FORMAT_CODE_LENGTH
                            );
        int format_code = (int)format_code_buffer[0];
        if(format_code > 5 || format_code == 3) {
            return ParseResult::ERROR;
        }
        section_header.setFormatCode(static_cast<RecordFormat>(format_code));
        
            // read frame name
        char frame_name_buffer[FILE_HEADER_FRAME_NAME_LENGTH + 1] = {0};
        header_start_index = *byte_index + FILE_HEADER_FRAME_NAME_START;
        this->_getFileBytes(
                            frame_name_buffer,
                            header_start_index,
                            FILE_HEADER_FRAME_NAME_LENGTH
                            );
        section_header.setFrameName(std::string(frame_name_buffer));
        
            // read company name
        char company_name_buffer[FILE_HEADER_COMPANY_NAME_LENGTH + 1] {0};
        header_start_index = *byte_index + FILE_HEADER_COMPANY_NAME_START;
        this->_getFileBytes(
                            company_name_buffer,
                            header_start_index,
                            FILE_HEADER_COMPANY_NAME_LENGTH
                            );
        section_header.setCompanyName(std::string(company_name_buffer));
        
            // read number of records
        char num_records_buffer[FILE_HEADER_NUMBER_OF_RECODRS_LENGTH] = {0};
        header_start_index = *byte_index + FILE_HEADER_NUMBER_OF_RECODRS_START;
        this->_getFileBytes(
                            num_records_buffer,
                            header_start_index,
                            FILE_HEADER_NUMBER_OF_RECODRS_LENGTH
                            );
        uint16_t num_record = this->_parseUint16(num_records_buffer, sizeof(num_records_buffer));
        section_header.setDataRecordCount((size_t)num_record);
        
            // read frame number
        char frame_number_buffer[FILE_HEADER_FRAME_NUMBER_LENGTH] = {0};
        header_start_index = *byte_index + FILE_HEADER_FRAME_NUMBER_START;
        this->_getFileBytes(
                            frame_number_buffer,
                            header_start_index,
                            FILE_HEADER_FRAME_NUMBER_LENGTH
                            );
        
        uint16_t frame_number = this->_parseUint16(frame_number_buffer, sizeof(frame_number_buffer));
        section_header.setFrameNumber((size_t)frame_number);
        
            // read frames in sequence
        char frames_in_seq_buffer[FILE_HEADER_FRAMES_IN_SEQUENCE_LENGTH] = {};
        header_start_index = *byte_index + FILE_HEADER_FRAMES_IN_SEQUENCE_START;
        this->_getFileBytes(
                            frames_in_seq_buffer,
                            header_start_index,
                            FILE_HEADER_FRAMES_IN_SEQUENCE_LENGTH
                            );
        uint16_t frames_in_sequence = this->_parseUint16(frames_in_seq_buffer, sizeof(frames_in_seq_buffer));
        section_header.setFramesInSequence((size_t) frames_in_sequence);
        
            // read projector number
        char projector_number_buffer[1] = {0};
        header_start_index = *byte_index + FILE_HEADER_PROJECTOR_NUMBER_START;
        this->_getFileBytes(
                            projector_number_buffer,
                            header_start_index,
                            FILE_HEADER_PROJECTOR_NUMBER_LENGTH
                            );
        section_header.setProjectorNumber((size_t) projector_number_buffer[0]);
        return ParseResult::SUCCESS;
    };
    
        // Format 0: 3D Coordinates with Indexed Color; Record Size: 8 Bytes
    ParseResult IldaFileProcessor::_parseDataRecordFormat_0(IldaDataRecord &data_record, char* buffer) {
        int pos_x = this->_parseTwosComplement(buffer[0], buffer[1]);
        int pos_y = this->_parseTwosComplement(buffer[2], buffer[3]);
        int pos_z = this->_parseTwosComplement(buffer[4], buffer[5]);
        bool is_last_point = buffer[6] & 0b10000000;
        bool blanking = buffer[6] & 0b01000000;
        data_record.setPosX(pos_x);
        data_record.setPosY(pos_y);
        data_record.setPosZ(pos_z);
        data_record.setLastPoint(is_last_point);
        data_record.setBlanking(blanking);
        data_record.setColorIndex(buffer[7]);
        
        return ParseResult::SUCCESS;
    };
    
        // Format 1: 2D Coordinates with Indexed Color; Record Size: 6 Bytes
    ParseResult IldaFileProcessor::_parseDataRecordFormat_1(IldaDataRecord &data_record, char* buffer) {
        int pos_x = this->_parseTwosComplement(buffer[0], buffer[1]);
        int pos_y = this->_parseTwosComplement(buffer[2], buffer[3]);
        bool is_last_point = buffer[4] & 0b10000000;
        bool blanking = buffer[4] & 0b01000000;
        data_record.setPosX(pos_x);
        data_record.setPosY(pos_y);
        data_record.setLastPoint(is_last_point);
        data_record.setBlanking(blanking);
        data_record.setColorIndex(buffer[5]);
        
        return ParseResult::SUCCESS;
    };
    
        // Format 2: Color Palette; Record Size: 3 Bytes
    ParseResult IldaFileProcessor::_parseDataRecordFormat_2(IldaDataRecord &data_record, char* buffer) {
        data_record.setRed(buffer[0]);
        data_record.setGreen(buffer[1]);
        data_record.setBlue(buffer[2]);
        return ParseResult::SUCCESS;
    };
    
        //Format 4: 3D Coordinates with True Color; Record Size: 10 Bytes
    ParseResult IldaFileProcessor::_parseDataRecordFormat_4(IldaDataRecord &data_record, char* buffer) {
        int pos_x = this->_parseTwosComplement(buffer[0], buffer[1]);
        int pos_y = this->_parseTwosComplement(buffer[2], buffer[3]);
        int pos_z = this->_parseTwosComplement(buffer[4], buffer[5]);
        bool is_last_point = buffer[6] & 0b10000000;
        bool blanking = buffer[6] & 0b01000000;
        data_record.setPosX(pos_x);
        data_record.setPosY(pos_y);
        data_record.setPosZ(pos_z);
        data_record.setLastPoint(is_last_point);
        data_record.setBlanking(blanking);
        data_record.setRed(buffer[7]);
        data_record.setGreen(buffer[8]);
        data_record.setBlue(buffer[9]);
        
        return ParseResult::SUCCESS;
    };
    
        // Format 5: 2D Coordinates with True Color; Record Size: 8 Bytes
    ParseResult IldaFileProcessor::_parseDataRecordFormat_5(IldaDataRecord &data_record, char* buffer) {
        int pos_x = this->_parseTwosComplement(buffer[0], buffer[1]);
        int pos_y = this->_parseTwosComplement(buffer[2], buffer[3]);
        bool is_last_point = buffer[4] & 0b10000000;
        bool blanking = buffer[4] & 0b01000000;
        data_record.setPosX(pos_x);
        data_record.setPosY(pos_y);
        data_record.setLastPoint(is_last_point);
        data_record.setBlanking(blanking);
        data_record.setRed(buffer[5]);
        data_record.setGreen(buffer[6]);
        data_record.setBlue(buffer[7]);
        
        return ParseResult::SUCCESS;
    };
    
    
    void IldaFileProcessor::_getFileBytes(char* buffer, size_t start, size_t byte_count) {
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
    
    int IldaFileProcessor::_parseTwosComplement(char most_significant, char least_significant){
        short value = (most_significant << 8) | (least_significant & 0xff) ;
        return (int)value;
    };
    
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
