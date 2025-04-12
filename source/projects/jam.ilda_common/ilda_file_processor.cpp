    //
    //  ilda_file_processor.cpp
    //  jam.helios
    //
    //  Created by Jan Mech on 28/2/25.
    //

#include "ilda_file_processor.hpp"
namespace jam::ilda {
    
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
        this->_ilda_frames.clear();
        if(!this->fileLoaded()) {
            return ParseResult::NODATA;
        }
        if(this->_ilda_file.size() < FILE_HEADER_SIZE) { // We need at least one complete header
            this->clearFileData();
            return ParseResult::ERROR;
        }
        size_t byte_index = 0;
        while(true) {
            IldaFrame frame;
            ParseResult result = this->_extractFrame(frame, &byte_index);
            if(result != ParseResult::SUCCESS) {
                if(result == ParseResult::END_OF_FILE) {
                    parse_result = ParseResult::SUCCESS;
                    break;
                } else {
                    parse_result = result;
                    break;
                }
            } else {
                this->_ilda_frames.push_back(std::move(frame));
            }
        }
        
        return parse_result;
    }
    
    bool IldaFileProcessor::fileLoaded() {
        return this->_fileLoaded;
    }
    
    std::vector<IldaFrame> IldaFileProcessor::getFrames() {
        return this->_ilda_frames;
    }
    
    
    ParseResult parseFramesToChar(std::vector<unsigned char> &file_bytes, const std::vector<IldaFrame> &frames) {
        for(size_t i = 0; i < frames.size(); i++) {
            IldaFrame f = frames[i];
                // header bytes 1 – 4: ILDA string
            file_bytes.push_back(static_cast<unsigned char>('I'));
            file_bytes.push_back(static_cast<unsigned char>('L'));
            file_bytes.push_back(static_cast<unsigned char>('D'));
            file_bytes.push_back(static_cast<unsigned char>('A'));
                // header bytes 5 - 7: reserved
            for(size_t j = 0; j < 4; j++) {
                file_bytes.push_back(0);
            }
                // header byte 8: format code
            file_bytes.push_back(static_cast<unsigned char>(f.getHeader().getFormatCode()));
            
                // header bytes 9 – 16: frame name
                // at this point we rely on propely formatted frames names
            std::string frame_name = f.getHeader().getFrameName();
            if(frame_name.size() > 8) {
                frame_name.resize(8);
            }
            for(int n_index = 0; n_index < 8; n_index++) {
                unsigned char c = 0;
                if(frame_name.size() > n_index - 1) {
                    c = static_cast<unsigned char>(frame_name[n_index]);
                }
                file_bytes.push_back(c);
            }
            
                // herader bytes 17 – 24: company name
                // at this point we rely on propely formatted frames names
            std::string comp_name = f.getHeader().getCompanyName();
            if(comp_name.size() > 8) {
                comp_name.resize(8);
            }
            for(int n_index = 0; n_index < 8; n_index++) {
                unsigned char c = 0;
                if(comp_name.size() > n_index - 1) {
                    c = static_cast<unsigned char>(comp_name[n_index]);
                }
                file_bytes.push_back(c);
            }
            
               
            
                // TODO: check if endianness is correct
                // header bytes 25 – 26: Number of Records
            {
                uint16_t count = static_cast<uint16_t>(f.getHeader().getDataRecordCount());
                unsigned char lsb = (count & 0x00FF);
                unsigned char msb = ((count & 0xFF00) >> 8);
                file_bytes.push_back(msb);
                file_bytes.push_back(lsb);
            }
            
               
                // header bytes 27 – 28: frame number
                // TODO: check if endianness is correct
            {
            uint16_t count = static_cast<uint16_t>(f.getHeader().getFrameNumber());
            unsigned char lsb = (count & 0x00FF);
            unsigned char msb = ((count & 0xFF00) >> 8);
            file_bytes.push_back(msb);
            file_bytes.push_back(lsb);
            }
            
                // header bytes 29 – 30: frames in sequence
                // TODO: check if endianness is correct
            {
            uint16_t count = static_cast<uint16_t>(f.getHeader().getFramesInSequence());
            unsigned char lsb = (count & 0x00FF);
            unsigned char msb = ((count & 0xFF00) >> 8);
            file_bytes.push_back(msb);
            file_bytes.push_back(lsb);
            }
            
                // header byte 31: projector number - we only support single projector standard, allways 0
            file_bytes.push_back(0);
            
                // header byte 32: reseved
            file_bytes.push_back(0);
            
            // TODO: Continue here
 
        }
        return ParseResult::SUCCESS;
    };
    
    /* protected methods */
    ParseResult IldaFileProcessor::_extractFrame(IldaFrame &frame, size_t *byte_index) {
        IldaHeader frame_header;
        ParseResult header_parse_result = this->_parseFrameHeader(frame_header, byte_index);
        
            // From the ilda file specs:
            //
            // 4.2.6. Number of Records
            // [...]
            // If the number of records is 0, then this is to be taken as the end of file header and no more data will follow this header.
        
        if(frame_header.getDataRecordCount() == 0) {
            return ParseResult::END_OF_FILE;
        }
        if(header_parse_result != ParseResult::SUCCESS) {
            this->clearFileData();
            return header_parse_result;
        }
        size_t record_count = frame_header.getDataRecordCount();
        for (size_t record_index = 0; record_index < record_count; record_index++) {
            size_t record_byte_size = this->_getRecordByteSize(frame_header.getFormatCode());
            size_t record_start_index = *byte_index + FILE_HEADER_SIZE + (record_index * record_byte_size);
            char record_buffer[10] = {0};
            this->_getFileBytes(
                                record_buffer,
                                record_start_index,
                                record_byte_size
                                );
            ParseResult record_parse_result = ParseResult::SUCCESS;
            IldaDataRecord data_record;
            switch (frame_header.getFormatCode()) {
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
            frame.pushRecord(std::move(data_record));
        }
        
        
        size_t data_records_byte_size = frame_header.getDataRecordCount() * this->_getRecordByteSize(frame_header.getFormatCode());
        
        *byte_index = *byte_index + FILE_HEADER_SIZE + data_records_byte_size;
        frame.setHeader(std::move(frame_header));
        
        return (*byte_index >= this->_ilda_file.size()) ? ParseResult::END_OF_FILE : ParseResult::SUCCESS;
    };
    
    ParseResult IldaFileProcessor::_parseFrameHeader(IldaHeader &frame_header, size_t *byte_index) {
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
        frame_header.setFormatCode(static_cast<RecordFormat>(format_code));
        
            // read frame name
        char frame_name_buffer[FILE_HEADER_FRAME_NAME_LENGTH + 1] = {0};
        header_start_index = *byte_index + FILE_HEADER_FRAME_NAME_START;
        this->_getFileBytes(
                            frame_name_buffer,
                            header_start_index,
                            FILE_HEADER_FRAME_NAME_LENGTH
                            );
        frame_header.setFrameName(std::string(frame_name_buffer));
        
            // read company name
        char company_name_buffer[FILE_HEADER_COMPANY_NAME_LENGTH + 1] {0};
        header_start_index = *byte_index + FILE_HEADER_COMPANY_NAME_START;
        this->_getFileBytes(
                            company_name_buffer,
                            header_start_index,
                            FILE_HEADER_COMPANY_NAME_LENGTH
                            );
        frame_header.setCompanyName(std::string(company_name_buffer));
        
            // read number of records
        char num_records_buffer[FILE_HEADER_NUMBER_OF_RECODRS_LENGTH] = {0};
        header_start_index = *byte_index + FILE_HEADER_NUMBER_OF_RECODRS_START;
        this->_getFileBytes(
                            num_records_buffer,
                            header_start_index,
                            FILE_HEADER_NUMBER_OF_RECODRS_LENGTH
                            );
        uint16_t num_record = this->_parseUint16(num_records_buffer, sizeof(num_records_buffer));
        frame_header.setDataRecordCount((size_t)num_record);
        
            // read frame number
        char frame_number_buffer[FILE_HEADER_FRAME_NUMBER_LENGTH] = {0};
        header_start_index = *byte_index + FILE_HEADER_FRAME_NUMBER_START;
        this->_getFileBytes(
                            frame_number_buffer,
                            header_start_index,
                            FILE_HEADER_FRAME_NUMBER_LENGTH
                            );
        
        uint16_t frame_number = this->_parseUint16(frame_number_buffer, sizeof(frame_number_buffer));
        frame_header.setFrameNumber((size_t)frame_number);
        
            // read frames in sequence
        char frames_in_seq_buffer[FILE_HEADER_FRAMES_IN_SEQUENCE_LENGTH] = {};
        header_start_index = *byte_index + FILE_HEADER_FRAMES_IN_SEQUENCE_START;
        this->_getFileBytes(
                            frames_in_seq_buffer,
                            header_start_index,
                            FILE_HEADER_FRAMES_IN_SEQUENCE_LENGTH
                            );
        uint16_t frames_in_sequence = this->_parseUint16(frames_in_seq_buffer, sizeof(frames_in_seq_buffer));
        frame_header.setFramesInSequence((size_t) frames_in_sequence);
        
            // read projector number
        char projector_number_buffer[1] = {0};
        header_start_index = *byte_index + FILE_HEADER_PROJECTOR_NUMBER_START;
        this->_getFileBytes(
                            projector_number_buffer,
                            header_start_index,
                            FILE_HEADER_PROJECTOR_NUMBER_LENGTH
                            );
        frame_header.setProjectorNumber((size_t) projector_number_buffer[0]);
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
