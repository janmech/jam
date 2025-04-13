    //
    //  ilda_file_processor.hpp
    //  jam.helios
    //
    //  Created by Jan Mech on 27/2/25.
    //

#ifndef ilda_file_processor_h
#define ilda_file_processor_h

#include <vector>
#include <cstddef>
#include <memory>
#include "ilda_definitions.hpp"
#include "ilda_header.hpp"
#include "ilda_data_record.hpp"
#include "ilda_frame.hpp"

#define STATUS_BYTE_BLANKING 0b01000000
#define STATUS_BYTE_LAST_POINT 0b10000000

namespace jam::ilda {
    class IldaFileProcessor {
    public:
        IldaFileProcessor() {}
        IldaFileProcessor(std::vector<char> ilda_file) {
            this->_ilda_file = ilda_file;
        }
        
        void setFileData(std::vector<char>ilda_file);
        void clearFileData();
        ParseResult parseFileDataToFrames();
        bool fileLoaded();
        std::vector<IldaFrame> getFrames();
        ParseResult parseFramesToFileData(std::vector<unsigned char> &file_bytes, const std::vector<IldaFrame> &frames);
        
        
    protected:
        
        bool _fileLoaded = false;
        std::vector<char> _ilda_file;
        std::vector<IldaFrame> _ilda_frames;
        
        void          _getFileBytes(char* buffer, size_t start, size_t byte_count);
        
        std::uint16_t _charToUint16(char* bytes, std::size_t byte_count);
        void          _uint16tToChar(uint16_t value, unsigned char * bytes);
        int           _parseCharToTwosComplement(char masb, char lsb);
        void          _parseInt16ToChar(int16_t value, unsigned char * bytes);
        
            // helper methods to parse raw file bytes to jam::ilda::xxx data structure
        ParseResult   _extractFrame(IldaFrame &frame, size_t *byte_index);
        ParseResult   _parseCharToFrameHeader(IldaHeader &frame_header, size_t *byte_index);
        ParseResult   _parseCharToDataRecord_Format0(IldaDataRecord &data_record, char* buffer);
        ParseResult   _parseCharToDataRecord_Format1(IldaDataRecord &data_record, char* buffer);
        ParseResult   _parseCharToDataRecord_Format2(IldaDataRecord &data_record, char* buffer);
        ParseResult   _parseCharToDataRecord_Format4(IldaDataRecord &data_record, char* buffer);
        ParseResult   _parseCharToDataRecord_Format5(IldaDataRecord &data_record, char* buffer);
        
            // helper methods to parse jam::ilda::xxx data structure to raw file bytes
        void _parseHeaderToChar(IldaHeader &h, std::vector<unsigned char> &file_bytes);
        void _parseDataRecordToChar_Format4(IldaDataRecord &dr, std::vector<unsigned char> &file_bytes);
        void _parseDataRecordToChar_Format5(IldaDataRecord &dr, std::vector<unsigned char> &file_bytes);
        
        
        size_t _getRecordByteSize(RecordFormat format);
        
    };
}
#endif /* ilda_file_processor_h */
