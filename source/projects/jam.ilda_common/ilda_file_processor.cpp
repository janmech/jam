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
    
    // TODO: function signature is inconsistent with parseFramesToFileData. Change to make consistet
    ParseResult IldaFileProcessor::parseFileDataToFrames() {
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
    
    ParseResult IldaFileProcessor::parseFramesToFileData(std::vector<unsigned char> &file_bytes, const std::vector<IldaFrame> &frames) {
        if(frames.size() == 0) {
            return ParseResult::NODATA;
        }
        for(size_t i = 0; i < frames.size(); i++) {
            IldaFrame f = frames[i];
            IldaHeader h = f.getHeader();
            this->_parseHeaderToChar(f.getHeader(), file_bytes);
            f.reset();
            IldaDataRecord dr;
            while (f.getNext(&dr)) {
                if(h.getFormatCode() == RecordFormat::FORMAT_5) {
                    this->_parseDataRecordToChar_Format5(dr, file_bytes);
                }
                
            }
        }
            // add end of file header
        IldaFrame last      = frames.back();
        IldaHeader h_eof    = last.getHeader();
        h_eof.setFrameName("EOF");
        h_eof.setDataRecordCount(0);
        this->_parseHeaderToChar(h_eof, file_bytes);
        
        return ParseResult::SUCCESS;
    };
    
    /* protected methods */
    
    void IldaFileProcessor::_parseHeaderToChar(IldaHeader &h, std::vector<unsigned char> &file_bytes) {
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
        file_bytes.push_back(static_cast<unsigned char>(h.getFormatCode()));
        
            // header bytes 9 – 16: frame name
            // at this point we rely on propely formatted frames names
        std::string frame_name = h.getFrameName();
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
        std::string comp_name = h.getCompanyName();
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
        
            // buffer for parsing uint16_t to msb/lsb
        unsigned char u16Bytes[2] = {0, 0};
        uint16_t u16Value = 0;
        
            // TODO: check if endianness is correct
            // header bytes 25 – 26: Number of Records
        u16Value = static_cast<uint16_t>(h.getDataRecordCount());
        this->_uint16tToChar(u16Value, u16Bytes);
        file_bytes.push_back(u16Bytes[0]);
        file_bytes.push_back(u16Bytes[1]);
        
            // header bytes 27 – 28: frame number
        
        u16Value = static_cast<uint16_t>(h.getFrameNumber());
        this->_uint16tToChar(u16Value, u16Bytes);
        file_bytes.push_back(u16Bytes[0]);
        file_bytes.push_back(u16Bytes[1]);
        
            // header bytes 29 – 30: frames in sequence
        u16Value = static_cast<uint16_t>(h.getFramesInSequence());
        file_bytes.push_back(u16Bytes[0]);
        file_bytes.push_back(u16Bytes[1]);
        
            // header byte 31: projector number - we only support single projector standard, allways 0
        file_bytes.push_back(0);
        
            // header byte 32: reseved
        file_bytes.push_back(0);
        
    };
    
    void IldaFileProcessor::_parseDataRecordToChar_Format5(IldaDataRecord &dr, std::vector<unsigned char> &file_bytes) {
        unsigned char bytes[2] = {0, 0};
        int16_t value = 0;
        
        // add x-coordinate
        value = static_cast<int16_t>(dr.getPosX());
        this->_parseInt16ToChar(value, bytes);
        file_bytes.push_back(bytes[0]);
        file_bytes.push_back(bytes[1]);
        
        // add x-coordinate
        value = static_cast<int16_t>(dr.getPosY());
        this->_parseInt16ToChar(value, bytes);
        file_bytes.push_back(bytes[0]);
        file_bytes.push_back(bytes[1]);
        
        // adding status byte
        unsigned char status_byte = 0;
        if(dr.getBlanking()) {
            status_byte = status_byte | STATUS_BYTE_BLANKING;
        }
        if(dr.getLastPoint()) {
            status_byte = status_byte | STATUS_BYTE_LAST_POINT;
        }
        file_bytes.push_back(status_byte);
        
        file_bytes.push_back(dr.getBlue());
        file_bytes.push_back(dr.getGreen());
        file_bytes.push_back(dr.getRed());
        
    };
    
    ParseResult IldaFileProcessor::_extractFrame(IldaFrame &frame, size_t *byte_index) {
        IldaHeader frame_header;
        ParseResult header_parse_result = this->_parseCharToFrameHeader(frame_header, byte_index);
        
            // From the ilda file specs:
            // 4.2.6. Number of Records
            // If the number of records is 0, then this is to be taken as the end of file header
            // and no more data will follow this header.
        
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
                    record_parse_result = this->_parseCharToDataRecord_Format0(data_record, record_buffer);
                    break;
                case RecordFormat::FORMAT_1 :
                    record_parse_result = this->_parseCharToDataRecord_Format1(data_record, record_buffer);
                    break;
                case RecordFormat::FORMAT_2 :
                    record_parse_result = this->_parseCharToDataRecord_Format2(data_record, record_buffer);
                    break;
                case RecordFormat::FORMAT_4 :
                    record_parse_result = this->_parseCharToDataRecord_Format4(data_record, record_buffer);
                    break;
                case RecordFormat::FORMAT_5 :
                    record_parse_result = this->_parseCharToDataRecord_Format5(data_record, record_buffer);
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
    
    ParseResult IldaFileProcessor::_parseCharToFrameHeader(IldaHeader &frame_header, size_t *byte_index) {
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
        uint16_t num_record = this->_charToUint16(num_records_buffer, sizeof(num_records_buffer));
        frame_header.setDataRecordCount((size_t)num_record);
        
            // read frame number
        char frame_number_buffer[FILE_HEADER_FRAME_NUMBER_LENGTH] = {0};
        header_start_index = *byte_index + FILE_HEADER_FRAME_NUMBER_START;
        this->_getFileBytes(
                            frame_number_buffer,
                            header_start_index,
                            FILE_HEADER_FRAME_NUMBER_LENGTH
                            );
        
        uint16_t frame_number = this->_charToUint16(frame_number_buffer, sizeof(frame_number_buffer));
        frame_header.setFrameNumber((size_t)frame_number);
        
            // read frames in sequence
        char frames_in_seq_buffer[FILE_HEADER_FRAMES_IN_SEQUENCE_LENGTH] = {};
        header_start_index = *byte_index + FILE_HEADER_FRAMES_IN_SEQUENCE_START;
        this->_getFileBytes(
                            frames_in_seq_buffer,
                            header_start_index,
                            FILE_HEADER_FRAMES_IN_SEQUENCE_LENGTH
                            );
        uint16_t frames_in_sequence = this->_charToUint16(frames_in_seq_buffer, sizeof(frames_in_seq_buffer));
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
    ParseResult IldaFileProcessor::_parseCharToDataRecord_Format0(IldaDataRecord &data_record, char* buffer) {
        int pos_x = this->_parseCharToTwosComplement(buffer[0], buffer[1]);
        int pos_y = this->_parseCharToTwosComplement(buffer[2], buffer[3]);
        int pos_z = this->_parseCharToTwosComplement(buffer[4], buffer[5]);
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
    ParseResult IldaFileProcessor::_parseCharToDataRecord_Format1(IldaDataRecord &data_record, char* buffer) {
        int pos_x = this->_parseCharToTwosComplement(buffer[0], buffer[1]);
        int pos_y = this->_parseCharToTwosComplement(buffer[2], buffer[3]);
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
    ParseResult IldaFileProcessor::_parseCharToDataRecord_Format2(IldaDataRecord &data_record, char* buffer) {
        data_record.setRed(buffer[0]);
        data_record.setGreen(buffer[1]);
        data_record.setBlue(buffer[2]);
        return ParseResult::SUCCESS;
    };
    
        //Format 4: 3D Coordinates with True Color; Record Size: 10 Bytes
    ParseResult IldaFileProcessor::_parseCharToDataRecord_Format4(IldaDataRecord &data_record, char* buffer) {
        int pos_x = this->_parseCharToTwosComplement(buffer[0], buffer[1]);
        int pos_y = this->_parseCharToTwosComplement(buffer[2], buffer[3]);
        int pos_z = this->_parseCharToTwosComplement(buffer[4], buffer[5]);
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
    ParseResult IldaFileProcessor::_parseCharToDataRecord_Format5(IldaDataRecord &data_record, char* buffer) {
        int pos_x = this->_parseCharToTwosComplement(buffer[0], buffer[1]);
        int pos_y = this->_parseCharToTwosComplement(buffer[2], buffer[3]);
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
    
    std::uint16_t IldaFileProcessor::_charToUint16(char* bytes, std::size_t byte_count) {
        uint16_t parsedInt = 0;
        switch (byte_count) {
            case 0:
                parsedInt = 0;
                break;
            case 1:
                parsedInt = (uint16_t)bytes[0];
                break;
            default:
                uint8_t msb = bytes[0];
                uint8_t lsb = bytes[1];
                parsedInt = ((uint16_t)msb) << 8 | (uint16_t)lsb;
                break;
        }
        
        return parsedInt;
    }
    
    void IldaFileProcessor::_uint16tToChar(uint16_t value, unsigned char * bytes) {
        unsigned char lsb = static_cast<unsigned char>(value & 0x00FF);
        unsigned char msb = static_cast<unsigned char>((value & 0xFF00) >> 8);
        bytes[0] = msb;
        bytes[1] = lsb;
    };
    
    int IldaFileProcessor::_parseCharToTwosComplement(char msb, char lsb){
        short value = (msb << 8) | (lsb & 0xff) ;
        return (int)value;
    };
    
    void  IldaFileProcessor::_parseInt16ToChar(int16_t value, unsigned char * bytes) {
        unsigned char lsb  = static_cast<uint8_t>(value & 0xFF);        // lower 8 bits
        unsigned char msb  = static_cast<uint8_t>((value >> 8) & 0xFF); // upper 8 bits
        bytes[0] = msb;
        bytes[1] = lsb;
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
