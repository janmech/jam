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

namespace jam::ilda {
    class IldaFileProcessor {
    public:
        IldaFileProcessor() {}
        IldaFileProcessor(std::vector<char> ilda_file) {
            this->_ilda_file = ilda_file;
        }
        
        void setFileData(std::vector<char>ilda_file);
        void clearFileData();
        ParseResult parseFileData();
        bool fileLoaded();
        std::vector<IldaFrame> getFrames();
        
        
    protected:
        
        bool _fileLoaded = false;
        std::vector<char> _ilda_file;
        std::vector<IldaFrame> _ilda_frames;
        
        void _getFileBytes(char* buffer, size_t start, size_t byte_count);
        std::uint16_t _parseUint16(char* bytes, std::size_t byte_count);
        int _parseTwosComplement(char most_significant, char least_significant); // value = (highbyte << 8) + lowbyte
        ParseResult _extractFrame(IldaFrame &frame, size_t *byte_index);
        ParseResult _parseFrameHeader(IldaHeader &frame_header, size_t *byte_index);
        ParseResult _parseDataRecordFormat_0(IldaDataRecord &data_record, char* buffer);
        ParseResult _parseDataRecordFormat_1(IldaDataRecord &data_record, char* buffer);
        ParseResult _parseDataRecordFormat_2(IldaDataRecord &data_record, char* buffer);
        ParseResult _parseDataRecordFormat_4(IldaDataRecord &data_record, char* buffer);
        ParseResult _parseDataRecordFormat_5(IldaDataRecord &data_record, char* buffer);
        size_t _getRecordByteSize(RecordFormat format);
        
    };
}
#endif /* ilda_file_processor_h */
